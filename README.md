# ShallotsOS PCSX4ALL

The PlayStation 1 emulator of [ShallotsOS](https://github.com/MarsInsanity/ShallotsOS)
for the SF3000 family (R36SX, SF3000, SF3500, GB350...). It is a standalone
emulator with a fast MIPS recompiler, forked from TreeFrogUI's SF3000 port
([tzubertowski/TreeFrogUI_pcsx4all](https://github.com/tzubertowski/TreeFrogUI_pcsx4all)).

## Works like the other systems

ShallotsOS runs its other systems in picoarch. This build behaves the same
way, so PS1 games feel no different:

- **The same in-game menu as picoarch.** It has the same layout, colours,
  theme font and wording:
  - Continue
  - Save / Load: pick a slot with Left/Right over the slot's screenshot.
    There are 30 slots, plus Auto and Switcher.
  - Disc control (multi-disc games)
  - Cheats (when there are any)
  - Options: Audio and video, Emulator options, Player controls, Save config
  - Reset
  - Exit Game

  PCSX's own settings pages use picoarch's list layout, and B goes back.
- **The same keys.** These follow FrogUI's *In-game Menu Keys* setting:
  - FN: tap for the menu, hold for the GameSwitcher.
  - Consoles without FN: SELECT+START, tap for the menu, hold for the
    GameSwitcher.
  - Or hold SELECT for the menu and hold START for the GameSwitcher.
  - SELECT+R2 / SELECT+L2 save and load the current slot. A toast confirms it.
- **GameSwitcher, Resume After Power-Off, Auto-Save/Load.** These use the
  same files and markers as picoarch:
  - the GameSwitcher picture in `picoarch/<folder>/<game>.scr.bmp`;
  - the Switcher state with its `.resume` marker;
  - `picoarch/resume_on_boot.txt`.

  So FrogUI's GameSwitcher, resume after power-off (power button held on the
  R36SX, or the shutdown's SIGTERM) and FrogUI's *Auto-Save/Load* setting work
  unchanged.
- **ShallotsOS sleep.** With *Disable Sleep* on, a short power press turns
  the screen off until the next press. After *Sleep Power-Off* it saves and
  powers off.

Save states are in `cubegm/cores/.pcsx4all/savestates/` as `<disc
ID>.<slot>.sav`. Each has a `.bmp` screenshot beside it. Slot 1 is the same
file the old menu used.

## Build

```sh
./build_sf3000.sh      # -> build/pcsx4all
```

This repository holds only the SF3000 / ShallotsOS files. The script
fetches the rest of the emulator from [gameblabla/pcsx4all](https://github.com/gameblabla/pcsx4all)
at the commit in `UPSTREAM`, lays these files over it, and builds with the
SF3000 toolchain. The toolchain is expected in `~/sf3000-work/sf3000toolchain`,
the same place ShallotsOS's other builds use. ShallotsOS's overlay workflow
builds it this way and installs it as `cubegm/pcsx4all`.

## License

PCSX4ALL is GPL v2 (PCSX, PCSX-Reloaded, PCSX ReARMed, the gpu_unai and SPU
plugins and their authors); TreeFrogUI's and ShallotsOS's changes are under
the same terms.
