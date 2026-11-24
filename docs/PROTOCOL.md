# CC-USB wire protocol

What `libccusb` sends, and where each detail comes from. "libxxusb" is
W-IE-NE-R's own library (version 3.x); "NSCLDAQ" is FRIB/NSCL's `CCCUSB`
driver (`main/usb/ccusb`, `main/usb/usb1/XXUSBUtil.cpp`).

## USB

| item | value | source |
|---|---|---|
| vendor / product | `0x16DC` / `0x0001` (VM-USB is `0x000B`) | both |
| configuration / interface | 1 / 0 | both |
| bulk OUT / IN endpoints | `0x02` / `0x86` | both |

All words are 16-bit little endian. The first word of every OUT packet is a
*target address* (TA).

## Packets

| purpose | OUT packet | reply |
|---|---|---|
| execute stack immediately | `000C, n, w1 … wn` | data words |
| load data stack | `0006, n, w1 … wn` | none |
| load scaler stack | `0007, n, w1 … wn` | none |
| read back stack | `0002` (data) / `0003` (scaler) | stack words (raw) |
| write action register | `0005, 000A, value` | none |

TA bits: `0x1` scaler stack, `0x2` stack select, `0x4` write, `0x8`
immediate (NSCLDAQ `XXUSBUtil.cpp`; libxxusb uses `(addr & 0x33) + 4`).

## Stack words

```text
NAF word   bit 15 CONT | bit 14 LONG | N[13:9] | A[8:5] | F[4:0]
```

* `LONG` (0x4000): 24-bit transfer. Write data then takes two words
  (`d15..d0`, `d23..d16`).
* `CONT` (0x8000): a mode word follows. Mode bits: `0x10` Q-stop, `0x20`
  Q-scan, `0x40` repeat (each followed by a count word), `0x80` wait for LAM,
  `0x100` fast CAMAC, `0x200` address pattern. Bit 15 of the mode word is set
  when a count follows.
* A write to N=0 (`F16`) puts its data word into the event as a marker.

## Replies

| operation | reply words |
|---|---|
| 24-bit read | `d15..d0`, then `X<<9 \| Q<<8 \| d23..d16` |
| 16-bit read | `d15..d0` (no Q/X) |
| write / control | `X<<1 \| Q` |
| N=25 register read (LONG) | `v15..v0`, `v31..v16` (full 32 bits) |

## Controller

| operation | NAF | note |
|---|---|---|
| Z (initialise) | N28 A8 F29 | libxxusb; NSCLDAQ swaps A8/A9 in its names |
| C (clear) | N28 A9 F29 | libxxusb |
| set / clear inhibit | N29 A9 F24 / F26 | both |
| inhibit state | N25 A10 F0, bit 24 | NSCLDAQ, firmware ≥ 0x8E000601 |
| registers | N25 A0–A14, F0 read / F16 write | both |

Action register bits: `0x1` start DAQ, `0x2` USB trigger, `0x4` clear,
`0x10` scaler dump (NSCLDAQ `CCCUSB.h`).

## Data buffers (DAQ mode)

```text
buffer header  bit15 last | bit14 scaler | bit13 continuous |
               bit12 multi-buffer | [11:0] number of events
(second header word if global mode bit 0x100 is set)
event header   bit13 stack id | bit12 continued | [11:0] words (excluding header)
...
0xFFFF         buffer terminator
```

From NSCLDAQ `DataBuffer.h` / `COutputThread.cpp`.
