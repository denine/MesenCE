#pragma once
#include "pch.h"
#include "NES/BaseMapper.h"
#include "NES/NesConsole.h"
#include "NES/NesCpu.h"
#include "NES/NesPpu.h"
#include "NES/Debugger/IExtModeMapperDebug.h"
#include "Utilities/Serializer.h"

/*
 * PRG layout: $5000 4KB RAM window, $6000 and $A000 16KB switchable,
 * $E000 fixed last 8KB. Bank write slots: 00=$6000, 01=$A000, 11=RAM, 10 unused.
 * Custom mapper, steps 1-2: PRG/CHR banking, PRG readback, cycle IRQ with
 * elapsed-cycle readback (dejitter).
 *
 * NOTE: written from memory of the Mesen2 BaseMapper API. If something does
 * not compile, compare names against Core/NES/BaseMapper.h and a small
 * existing mapper (for example Mapper183.h or Bmc190in1.h).
 *
 * Needs an NES 2.0 header with: PRG-ROM 512KB, CHR-ROM 0, CHR-RAM 128KB,
 * battery PRG-RAM 32KB. RENDER bit 7 picks the 64KB CHR-RAM half the PPU sees.
 * Header: 4E 45 53 1A 20 00 02 A8 0F 00 90 0B 00 00 00 00 Register it in MapperFactory.cpp with an unused ID.
 *
 * Step 3 done: MIRROR (nametables mapped onto CHR-RAM pages, mirror-all).
 * Step 4 done: extended mode (RENDER bit1 = 1, bit0 = 0): the byte at the same
 * position in page P|1 gives the tile's palette (bits 1-0) and a 4KB BG bank
 * (bits 4-2). BG banks come from the first 32KB of CHR-RAM; in extended mode
 * the CPU/sprite side ($0000-$1FFF) maps the last 32KB. Split intervals were
 * dropped, RENDER bits 7-3 are reserved.
 * If your Mesen2 version needs a flag to enable custom VRAM reads, set it in
 * InitMapper (see how MMC5.h does it).
 * Step 5 done: Big Tile (RENDER bit0 = 1), alone or with extended mode.
 * With extended on, the nametable byte's bits 1-0 pick a 4KB BG bank (0-3).
 */
class CustomMapper : public BaseMapper, public IExtModeMapperDebug
{
private:
	// PRG regs hold the BANK write format: bit7=0, bits6-5=slot, bits4-0=bank
	uint8_t _prgReg[4] = {};
	uint8_t _chrReg[4] = {};
	uint8_t _mirrorReg[4] = {};
	uint8_t _render = 0;

	// IRQ: one-shot cycle counter. A write to $4805 loads and starts it.
	uint8_t _irqLatchLow = 0;
	uint16_t _irqCounter = 0;
	bool _irqRun = false;
	bool _irqPending = false; // drives /IRQ low while true
	bool _irqFired = false;   // elapsed counter runs after the first fire
	uint8_t _irqElapsed = 0;  // 4 bits, wraps

	// Extended mode fetch tracking (transient, not saved)
	// 0 = idle, 1 = NT fetch just seen, 2 = BG pattern fetches expected
	uint8_t _fetchState = 0;
	uint8_t _patLeft = 0;
	uint16_t _ntAddr = 0;  // $2000-$2FFF address of the last NT fetch
	bool _bgPhase = false; // last NT fetch belongs to a BG tile, not sprites
	uint8_t _extBank = 0;  // 8KB CHR bank for this tile's pattern fetches
	bool _useBank = false; // pattern fetches of this tile use _extBank
	uint8_t _bigByte = 0;  // Big Tile nametable byte of the current tile

	void StopIrq()
	{
		_irqRun = false;
		_irqPending = false;
		_irqFired = false;
		_irqElapsed = 0;
		_console->GetCpu()->ClearIrqSource(IRQSource::External);
	}

	void StartIrq(uint8_t high)
	{
		_irqCounter = (high << 8) | _irqLatchLow;
		_irqRun = true;
		_irqPending = false;
		_irqFired = false;
		_irqElapsed = 0;
		_console->GetCpu()->ClearIrqSource(IRQSource::External);
	}

	// RENDER bit 7 selects which 64KB half of the 128KB CHR-RAM chip the PPU
	// sees (CHR slots, nametables, extended pages, per-tile BG banks).
	// On hardware this is just the RAM's A16 pin.
	uint32_t ChrHalfPages() { return (_render & 0x80) ? 64 : 0; }
	uint32_t ChrHalfBytes() { return (_render & 0x80) ? 0x10000 : 0; }

	// Each nametable maps a 1KB page of CHR-RAM (64 pages per half)
	void UpdateNametables()
	{
		for(int i = 0; i < 4; i++) {
			uint16_t start = 0x2000 + i * 0x400;
			SetPpuMemoryMapping(
				start, start + 0x3FF, ChrMemoryType::ChrRam,
				((_mirrorReg[i] & 0x3F) + ChrHalfPages()) * 0x400,
				MemoryAccessType::ReadWrite
			);
		}
	}

	void WriteMirror(uint8_t value)
	{
		uint8_t page = value & 0x3F;
		if(_render & 0x01) {
			// Big Tile / mirror-all: bits 7-6 ignored, all four load
			for(int i = 0; i < 4; i++) {
				_mirrorReg[i] = page;
			}
		} else {
			_mirrorReg[(value >> 6) & 0x03] = page;
		}
		UpdateNametables();
	}

	// Latches with no reset on the real hardware. Follows Mesen's RAM power-on
	// setting (all 0s / all 1s / random). If this name doesn't exist in your
	// version, replace the body with any random byte source.
	uint8_t RandomByte()
	{
		return GetPowerOnByte();
	}

	void MapRom(uint16_t start, uint16_t end, uint32_t offset)
	{
		SetCpuMemoryMapping(start, end, PrgMemoryType::PrgRom, offset, MemoryAccessType::Read);
	}

	void UpdatePrg()
	{
		// Two switchable 16KB windows, last 8KB of ROM fixed at $E000
		MapRom(0x6000, 0x9FFF, (_prgReg[0] & 0x1F) * 0x4000);
		MapRom(0xA000, 0xDFFF, (_prgReg[1] & 0x1F) * 0x4000);
		MapRom(0xE000, 0xFFFF, 0x7E000);

		bool writeProtect = (_render & 0x04) != 0;
		SetCpuMemoryMapping(
			0x5000, 0x5FFF, PrgMemoryType::SaveRam,
			(_prgReg[3] & 0x07) * 0x1000,
			writeProtect ? MemoryAccessType::Read : MemoryAccessType::ReadWrite
		);
	}

	void UpdateChr()
	{
		if(_render & 0x02) {
			// Sprite/CPU side lives in the LAST 32KB of CHR-RAM (pages 32-63).
			// BG tiles in extended mode live in the FIRST 32KB.
			// $0000-$0FFF: fixed to the first 4KB of the upper half.
			// $1000-$1FFF: four 1KB slots, 5-bit bank within the upper half.
			uint32_t h = ChrHalfPages();
			for(int i = 0; i < 4; i++) {
				SelectChrPage(i, h + 32 + i, ChrMemoryType::ChrRam);
				SelectChrPage(4 + i, h + 32 + (_chrReg[i] & 0x1F), ChrMemoryType::ChrRam);
			}
		} else {
			// 2KB mode: four 2KB slots, bank counted in 2KB units
			for(int i = 0; i < 4; i++) {
				SelectChrPage2x(i, ChrHalfPages() + _chrReg[i] * 2, ChrMemoryType::ChrRam);
			}
		}
	}

	void WriteBank(uint8_t value)
	{
		uint8_t slot = (value >> 5) & 0x03;
		if(value & 0x80) {
			_chrReg[slot] = value & 0x1F;
			UpdateChr();
		} else {
			// Store in readback format. RAM window only keeps bits 2-0.
			if(slot == 2) {
				return; // unused slot code
			}
			uint8_t bank = (slot == 3) ? (value & 0x07) : (value & 0x1F);
			_prgReg[slot] = (slot << 5) | bank;
			UpdatePrg();
		}
	}

protected:
	uint16_t GetPrgPageSize() override { return 0x2000; }
	uint16_t GetChrPageSize() override { return 0x400; }
	uint32_t GetChrRamSize() override { return 0x20000; }
	uint32_t GetSaveRamSize() override { return 0x8000; }
	uint32_t GetWorkRamSize() override { return 0; }
	uint16_t RegisterStartAddress() override { return 0x4800; }
	uint16_t RegisterEndAddress() override { return 0x4805; }
	// Without this, reads of $4800-$4805 return open bus instead of ReadRegister
	bool AllowRegisterRead() override { return true; }
	// Without this, NesConsole never calls ProcessCpuClock, so the IRQ counter never runs
	bool EnableCpuClockHook() override { return true; }
	// Without this the base class never calls MapperReadVram, so extended
	// and Big Tile modes do nothing
	bool EnableCustomVramRead() override { return true; }

	void InitMapper() override
	{
		// Power-on: RENDER = 0, everything else defined here
		// (real hardware leaves the rest random)
		_render = 0;
		// Slot 2 is unused; $E000 is fixed so nothing needs a defined value
		_prgReg[0] = 0x00 | (RandomByte() & 0x1F);
		_prgReg[1] = 0x20 | (RandomByte() & 0x1F);
		_prgReg[2] = 0x40;
		_prgReg[3] = 0x60 | (RandomByte() & 0x07);
		for(int i = 0; i < 4; i++) {
			_chrReg[i] = RandomByte() & 0x1F;
			_mirrorReg[i] = RandomByte() & 0x3F;
		}
		UpdatePrg();
		UpdateChr();
		UpdateNametables();
	}

	bool ExtendedActive() { return (_render & 0x03) == 0x02; }

	// The fetch hook runs in extended, Big Tile and both together
	bool FetchHookActive() { return (_render & 0x03) != 0; }

	// Big Tile: one 1KB page holds four 256-byte screens. Offset inside the
	// page = {A11,A10 screen} * $100 + {A9-A6 row>>1} * 16 + {A4-A1 col>>1}
	uint32_t BigTileOffset(uint16_t ntAddr)
	{
		return ((ntAddr >> 10) & 0x03) * 0x100 + ((ntAddr >> 6) & 0x0F) * 16 + ((ntAddr >> 1) & 0x0F);
	}

	uint8_t Replicate(uint8_t p)
	{
		p &= 0x03;
		return p | (p << 2) | (p << 4) | (p << 6);
	}

	// Attribute byte for a Big Tile. Row parity = A5, column parity = A0.
	uint8_t BigTileAttribute()
	{
		uint8_t rowParity = (_ntAddr >> 5) & 1;
		uint8_t colParity = _ntAddr & 1;
		if(_render & 0x02) {
			// Big Tile + extended: palette byte from page P|1, bank from the NT byte
			uint8_t nt = (_ntAddr >> 10) & 0x03;
			uint32_t extPage = (_mirrorReg[nt] & 0x3F) | 1;
			uint8_t paletteByte = ReadChrRamRaw(extPage * 0x400 + BigTileOffset(_ntAddr));
			_extBank = _bigByte & 0x03;
			_useBank = true;
			// Original NES field order: TL bits 1-0, TR 3-2, BL 5-4, BR 7-6
			return Replicate(paletteByte >> (((rowParity << 1) | colParity) * 2));
		}
		// Plain Big Tile: palette is the NT byte's bits 1-0
		return Replicate(_bigByte);
	}

	// EMULATOR SHORTCUT: the real mapper cannot see the PPU dot. It must tell
	// BG tiles from sprite fetches by watching NT addresses (sprite garbage
	// fetches repeat the same NT address). Replace this when moving to hardware.
	bool IsBgFetchPhase()
	{
		uint16_t cycle = _console->GetPpu()->GetCurrentCycle();
		return (cycle >= 1 && cycle <= 256) || (cycle >= 321 && cycle <= 336);
	}

	uint8_t ReadChrRamRaw(uint32_t offset)
	{
		return _chrRam[(offset + ChrHalfBytes()) % _chrRamSize];
	}

	// Fetch order per tile: NT, AT, pattern low, pattern high.
	// Two A13=1 reads in a row = NT then AT. The AT read is replaced by the
	// extended byte (palette repeated in all four fields), and the next two
	// pattern reads use the 8KB bank from that byte.
	uint8_t MapperReadVram(uint16_t addr, MemoryOperationType type) override
	{
		if(type != MemoryOperationType::PpuRenderingRead || !FetchHookActive()) {
			_fetchState = 0;
			return BaseMapper::MapperReadVram(addr, type);
		}

		if(addr >= 0x2000 && addr <= 0x2FFF) {
			if(_fetchState == 1) {
				// AT fetch: directly after an NT fetch
				_fetchState = 2;
				_patLeft = 2;
				_useBank = false;
				if(_bgPhase && (_render & 0x01)) {
					return BigTileAttribute();
				}
				if(_bgPhase) {
					uint8_t nt = (_ntAddr >> 10) & 0x03;
					uint32_t extPage = (_mirrorReg[nt] & 0x3F) | 1;
					uint8_t value = ReadChrRamRaw(extPage * 0x400 + (_ntAddr & 0x3FF));
					// Extended byte: bits 4-2 = 4KB bank (first 32KB of CHR-RAM),
					// bits 1-0 = palette for this 8x8 tile
					_extBank = (value >> 2) & 0x07;
					_useBank = true;
					return Replicate(value);
				}
				return BaseMapper::MapperReadVram(addr, type);
			}

			// NT fetch. Also restarts a group that never got its pattern reads
			// (the two unused NT fetches at dots 337-340).
			_fetchState = 1;
			_ntAddr = addr & 0x0FFF;
			_bgPhase = IsBgFetchPhase();
			if(_bgPhase && (_render & 0x01)) {
				// Big Tile: one byte covers 2x2 tiles. Send {index, row parity,
				// column parity} as the tile ID, keep the byte for the AT fetch.
				uint8_t nt = (_ntAddr >> 10) & 0x03;
				_bigByte = ReadChrRamRaw((_mirrorReg[nt] & 0x3F) * 0x400 + BigTileOffset(_ntAddr));
				uint8_t parity = (((_ntAddr >> 5) & 1) << 1) | (_ntAddr & 1);
				return (_bigByte & 0xFC) | parity;
			}
			return BaseMapper::MapperReadVram(addr, type);
		}

		// Pattern fetch
		if(_fetchState == 2 && _patLeft > 0) {
			_patLeft--;
			if(_patLeft == 0) {
				_fetchState = 0;
			}
			if(_bgPhase && _useBank) {
				// 4KB bank inside the first 32KB (3 bits, or 2 bits in Big Tile).
				// The PPU's BG table bit (A12) is ignored.
				return ReadChrRamRaw(_extBank * 0x1000 + (addr & 0x0FFF));
			}
		} else {
			_fetchState = 0;
		}
		return BaseMapper::MapperReadVram(addr, type);
	}

	void Reset(bool softReset) override
	{
		BaseMapper::Reset(softReset);
		// Same state on power-on and soft reset: only the pieces that have a
		// defined power-on value. The other latches keep what they had.
		_render = 0;
		_fetchState = 0;
		_patLeft = 0;
		StopIrq();
		UpdatePrg();
		UpdateChr();
		UpdateNametables();
	}

	// Called once per CPU cycle (M2)
	void ProcessCpuClock() override
	{
		// The base class does its own per-cycle bookkeeping
		BaseMapper::ProcessCpuClock();

		// Elapsed counts cycles after the fire, so it advances before the
		// fire check: the fire cycle itself sets it to 0.
		if(_irqFired) {
			_irqElapsed = (_irqElapsed + 1) & 0x0F;
		}

		if(_irqRun) {
			if(_irqCounter > 0) {
				_irqCounter--;
			}
			// A reload of 0 or 1 fires on the first clock
			if(_irqCounter == 0) {
				_irqRun = false;
				_irqPending = true;
				_irqFired = true;
				_irqElapsed = 0;
				_console->GetCpu()->SetIrqSource(IRQSource::External);
			}
		}
	}

	uint8_t ReadRegister(uint16_t addr) override
	{
		if(addr >= 0x4800 && addr <= 0x4803) {
			return _prgReg[addr - 0x4800];
		}
		if(addr == 0x4804) {
			// Upper bits driven as 0, like the real mapper must do
			return _irqElapsed & 0x0F;
		}
		return BaseMapper::ReadRegister(addr);
	}

	void WriteRegister(uint16_t addr, uint8_t value) override
	{
		switch(addr) {
			case 0x4800: WriteBank(value); break;
			case 0x4801: WriteMirror(value); break;
			case 0x4802:
				_render = value;
				UpdatePrg();        // bit 2 changes RAM write protect
				UpdateChr();        // bits 1 and 7 change the CHR slot layout
				UpdateNametables(); // bit 7 changes the nametable pages
				break;
			case 0x4804:
				// Load latch low, acknowledge and cancel
				_irqLatchLow = value;
				StopIrq();
				break;
			case 0x4805:
				// Load counter from {value, latch low} and start
				StartIrq(value);
				break;
		}
	}

	void Serialize(Serializer& s) override
	{
		BaseMapper::Serialize(s);
		SVArray(_prgReg, 4);
		SVArray(_chrReg, 4);
		SVArray(_mirrorReg, 4);
		SV(_render);
		SV(_irqLatchLow);
		SV(_irqCounter);
		SV(_irqRun);
		SV(_irqPending);
		SV(_irqFired);
		SV(_irqElapsed);
		if(!s.IsSaving()) {
			UpdatePrg();
			UpdateChr();
			UpdateNametables();
			_fetchState = 0;
			_patLeft = 0;
		}
	}

public:
	// ---- Debugger support (tilemap viewer) ----
	// Mirrors what MapperReadVram does during rendering, so the viewer shows
	// the same picture. Only the four real nametables (0-3) are handled.

	// Big Tile byte for a tile (ntOffset = row * 32 + column inside nametable nt)
	uint8_t DebugBigByte(uint8_t nt, uint16_t ntOffset)
	{
		uint16_t ntAddr = (nt << 10) | (ntOffset & 0x3FF);
		return ReadChrRamRaw((_mirrorReg[nt] & 0x3F) * 0x400 + BigTileOffset(ntAddr));
	}

	// Byte from the extended page (P|1) at the same position as the tile
	uint8_t DebugExtByte(uint8_t nt, uint16_t ntOffset)
	{
		uint32_t extPage = (_mirrorReg[nt] & 0x3F) | 1;
		if(_render & 0x01) {
			uint16_t ntAddr = (nt << 10) | (ntOffset & 0x3FF);
			return ReadChrRamRaw(extPage * 0x400 + BigTileOffset(ntAddr));
		}
		return ReadChrRamRaw(extPage * 0x400 + (ntOffset & 0x3FF));
	}

	bool HasExtendedAttributes(ExtModeConfig& extCfg, uint8_t ntIndex) override
	{
		return ntIndex < 4 && (_render & 0x03) != 0;
	}

	bool HasExtendedBackground(ExtModeConfig& extCfg, uint8_t ntIndex) override
	{
		// Per-tile bank exists in extended mode and Big Tile + extended
		return ntIndex < 4 && (_render & 0x02) != 0;
	}

	bool HasTileIndexOverride(ExtModeConfig& extCfg, uint8_t ntIndex) override
	{
		return ntIndex < 4 && (_render & 0x01) != 0;
	}

	uint8_t GetTileIndexOverride(ExtModeConfig& extCfg, uint8_t ntIndex, uint16_t ntOffset) override
	{
		// {index (6 bits), row parity, column parity}
		uint8_t rowParity = (ntOffset >> 5) & 1;
		uint8_t colParity = ntOffset & 1;
		return (DebugBigByte(ntIndex, ntOffset) & 0xFC) | (rowParity << 1) | colParity;
	}

	uint8_t GetExAttributePalette(ExtModeConfig& extCfg, uint8_t ntIndex, uint16_t ntOffset) override
	{
		if(_render & 0x01) {
			if(_render & 0x02) {
				// Big Tile + extended: field picked by position in the 2x2 block
				uint8_t rowParity = (ntOffset >> 5) & 1;
				uint8_t colParity = ntOffset & 1;
				return (DebugExtByte(ntIndex, ntOffset) >> (((rowParity << 1) | colParity) * 2)) & 0x03;
			}
			// Plain Big Tile: palette is the nametable byte's bits 1-0
			return DebugBigByte(ntIndex, ntOffset) & 0x03;
		}
		// Extended: bits 1-0 of the extended byte
		return DebugExtByte(ntIndex, ntOffset) & 0x03;
	}

	uint8_t GetExBackgroundChrData(ExtModeConfig& extCfg, uint8_t ntIndex, uint16_t ntOffset, uint16_t chrAddr) override
	{
		uint8_t bank;
		if(_render & 0x01) {
			bank = DebugBigByte(ntIndex, ntOffset) & 0x03; // 2 bits, 4KB banks
		} else {
			bank = (DebugExtByte(ntIndex, ntOffset) >> 2) & 0x07; // 3 bits, 4KB banks
		}
		return ReadChrRamRaw(bank * 0x1000 + (chrAddr & 0x0FFF));
	}
};