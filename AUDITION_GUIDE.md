# Sound Audition / QA Mode (AUD)

This firmware build adds an **AUD** serial-command mode so you can quickly:

1) **Browse clips** (one at a time) and decide if they’re distinct enough  
2) **Simulate rounds** (common vs odd) using those clips  
3) **Tune per-clip loudness** by recording *suggested* dB deltas you can copy into the SD card `manifest.csv`

Important notes:
- **AUD does NOT modify the SD card**. It only keeps a temporary list of “suggested_delta_db” values in the Master’s RAM and applies them as temporary per-slot trims while auditioning.
- To make the changes permanent, you’ll edit `manifest.csv` on the SD card:  
  `new_volume_db = old_volume_db + suggested_delta_db`

---

## How to use

Open the **Master** Serial Monitor (same port as usual) and type commands on separate lines.

### Enter / exit
- `AUD ON`
- `AUD OFF`

### The 3 audition modes

#### Mode 1: CLIP (single-clip, single-speaker)
Plays **one clip** on **one speaker** (everything else silent).

- `AUD MODE CLIP`
- `AUD NEXT` / `AUD PREV` to step through clips in the MasterManifest list
- Choose the output speaker:
  - `AUD SOLOPOS A0`..`A3` or `B0`..`B3`
  - or `AUD SOLOPOS 0`..`7` (0-3 = A0-A3, 4-7 = B0-B3)

**Button shortcut (while AUD is ON):**
- In **CLIP** mode, pressing any button sets **SOLOPOS** to that speaker.

#### Mode 2: ROUND (odd-one-out simulation)
Plays **COMMON** on 7 speakers, **ODD** on 1 speaker.

- `AUD MODE ROUND`
- Set which speaker is the odd position:
  - `AUD ODDPOS A0`..`A3` or `B0`..`B3` or `0`..`7`
  - `AUD ROTATE` (moves odd position to next)
- Pick clips:
  - `AUD COMMON CUR` (use current cursor clip as common)
  - `AUD ODD NEXT` / `AUD ODD PREV` (step odd clip)
  - Or set explicit IDs:
    - `AUD COMMON 1234`
    - `AUD ODD 5678`

**Button shortcut:**
- In **ROUND** mode, pressing any button sets **ODDPOS** to that speaker.

#### Mode 3: SIM1 / SIM2 / SIM3 (real game scene simulation)
Generates and plays **real gameplay scenes** using the same logic as the game:

- `AUD MODE SIM1` (Level 1 logic)
- `AUD MODE SIM2` (Level 2 logic)
- `AUD MODE SIM3` (Level 3 logic)

Step to a new simulated scene:
- `AUD NEXT` / `AUD PREV` (both generate a new scene)
- Or press **any button** (shortcut = next simulated scene)

---

## Per-clip loudness tuning (TRIMSET + EXPORT)

### Set a suggested delta dB
This applies **temporarily** while auditioning *and* records the delta for later export.

- In **CLIP** mode (current clip):
  - `AUD TRIMSET -3`
- In **ROUND** mode:
  - `AUD TRIMSET ODD -2`
  - `AUD TRIMSET COMMON +1`
- In **SIM** mode (targets the odd clip in the current scene by default):
  - `AUD TRIMSET -2`

### Target a specific speaker position in the current scene
This is the most direct way to tune a specific clip **you are currently hearing**:

- `AUD TRIMSET SLOT A0 -3`
- `AUD TRIMSET SLOT B2 +2`
- `AUD TRIMSET SLOT 6 -4`

(Works in CLIP / ROUND / SIM.)

### Export your suggestions
- `AUD EXPORT`

It prints:

`id,delta_db,base,sub,sub2`

Then you update the SD `manifest.csv` for those IDs:
- `new_volume_db = old_volume_db + delta_db`

### Clear all suggestions
- `AUD CLEARTRIM`

---

## Filters (optional, for faster browsing)

These filters only affect the **cursor** used by `AUD NEXT/PREV` in CLIP/ROUND.

- `AUD BASE tones`  
- `AUD SUB sweep`  
- `AUD SUB2 up_short`  

Clear one filter:
- `AUD BASE CLEAR`  (or `SUB CLEAR`, `SUB2 CLEAR`)

Clear all filters:
- `AUD CLEARFILTER`

---

## Visual indicators during AUD

This build uses a “solid LED map” while auditioning:
- **Odd** = **GREEN**
- **Common** = **WHITE**
- **Silent / unused** = **OFF**
- CLIP mode: the active SOLOPOS speaker is **WHITE**; others are **OFF**

(These colors are for QA only; gameplay is unchanged.)
