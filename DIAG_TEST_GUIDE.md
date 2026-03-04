# Seashells DIAG AUDIO – Test Guide (Patterns 1–15)

This guide is for the **DIAG AUDIO (tone test)** mode built into the Seashells firmware.

The goal is to **separate two very different root causes** of “I hear two sounds at once”:

1) **Electrical mixing (L+R)**: a mono I2S amp that should be RIGHT-only is actually strapped/configured as **MIX**, so the RIGHT speaker outputs **L+R**.
2) **Acoustic bleed / cabinet coupling**: channels are electrically independent, but sound from other nearby speakers is physically audible.

Patterns **14/15** are designed to make this distinction *obvious*.

---

## 0) Speaker ↔ slot mapping used by DIAG

The Side firmware drives 4 independent mono channels:

- **Slot 0 / Speaker 1** = **I2S0 LEFT**
- **Slot 1 / Speaker 2** = **I2S0 RIGHT**
- **Slot 2 / Speaker 3** = **I2S1 LEFT**
- **Slot 3 / Speaker 4** = **I2S1 RIGHT**

In DIAG mode, the Side LEDs show what should be active:

- **WHITE** = tone active on this slot
- **GREEN** = “odd” (patterns 8–11)
- **RED** = “common” (patterns 8–11)
- **OFF** = silent

---

## 1) How to enter DIAG mode

Open the **Master** Serial monitor (115200 baud, Newline). Commands:

- `DIAG` → prints help
- `DIAG OFF` → exit diag
- `DIAG <n>` → pattern n on BOTH sides
- `DIAG A <n>` → only Side A (Side B forced OFF)
- `DIAG B <n>` → only Side B (Side A forced OFF)
- `DIAG NEXT` / `DIAG PREV` → step patterns
- `DIAG AUTO` → auto-cycle

**Tip:** While DIAG is enabled, **any button press** advances to the next pattern.

---

## 2) How to listen (important)

To avoid false conclusions:

1. Test **one side at a time** (`DIAG A ...` then `DIAG B ...`).
2. For “should be silent” checks, put your ear **1–2 inches from the cone/grille**.
3. If possible, **cover/muffle the other speaker in the same I2S pair** with a towel/hand when doing “silent” checks.

What “silent” means here:

- **PASS:** basically no clear pitch at the cone (maybe faint hiss)
- **FAIL:** you can clearly identify the tone pitch at the cone

---

## 3) MixFix (runtime) — what it is and how to control it

Some mono I2S amp boards can be strapped for **LEFT**, **RIGHT**, or **MIX (L+R)**.

If a RIGHT speaker amp is in MIX mode, the Side firmware can apply a **software MixFix**:

> `R_out = (k*R - m*L) >> 12`

Typical settings (Master command uses “milli” units):

- For MIX ≈ **(L+R)/2** (most common): `k=2000`, `m=1000`
- For MIX ≈ **(L+R)** (rare): `k=1000`, `m=1000`

Master commands:

- `MIXFIX` → help
- `MIXFIX ON`  → enable on BOTH sides (mask=3, k=2.0, m=1.0)
- `MIXFIX OFF` → disable on BOTH sides
- `MIXFIX A ON|OFF` / `MIXFIX B ON|OFF`
- `MIXFIX [A|B|BOTH] <mask 0-3> <k_milli> <m_milli>`
  - mask bit0 = I2S0 RIGHT (slot1)
  - mask bit1 = I2S1 RIGHT (slot3)

---

## 4) Pattern reference (what you should hear)

### Pair 0 (I2S0): Slot0 + Slot1

#### Pattern 1 – I2S0 LEFT only (440 Hz)
Command: `DIAG A 1`
- Slot0 LED = WHITE
- **Speaker1 (slot0):** loud, steady **440 Hz**
- **Speaker2 (slot1):** should be **silent at the cone**

#### Pattern 2 – I2S0 RIGHT only (880 Hz)
Command: `DIAG A 2`
- Slot1 LED = WHITE
- **Speaker2 (slot1):** loud, steady **880 Hz**
- **Speaker1 (slot0):** should be **silent at the cone**

#### Pattern 3 – I2S0 L+R together (440 Hz + 880 Hz)
Command: `DIAG A 3`
- Slot0 + Slot1 LEDs = WHITE
- **Speaker1:** should sound like **only 440** at the cone
- **Speaker2:** should sound like **only 880** at the cone

If a single speaker has BOTH tones strongly at its cone → that amp is mixing L+R.

#### Pattern 12 – PHASE-CANCEL I2S0 (slot0=+440, slot1=−440)
Command: `DIAG A 12`
- Slot0 + Slot1 LEDs = WHITE

**At the cone**, each speaker should still play a tone if channels are independent.
If a speaker becomes dramatically quieter/near-silent at the cone → likely **mono-summing (L+R)**.

⚠️ This test can be fooled by **acoustic cancellation** in the room, so treat it as supporting evidence.

#### Pattern 14 – MIXFIX TOGGLE (I2S0) — MOST DECISIVE
Command: `DIAG A 14`
- Slot0 LED = WHITE (tone)
- Slot1 LED alternates **RED ↔ GREEN**
  - **RED = MixFix OFF**
  - **GREEN = MixFix ON**

**Exactly what to listen for:**

Put your ear right at **Speaker2 (slot1)** while watching slot1 LED:

- If the tone at slot1 becomes **noticeably quieter on GREEN** (and louder on RED):
  - Speaker2 is almost certainly **electrically mixing L+R**.
  - MixFix should help in gameplay.

- If the tone at slot1 becomes **much louder on GREEN**:
  - Speaker2 is likely **NOT** mixing L+R (it’s RIGHT-only).
  - What you were hearing before is mostly **acoustic bleed**.
  - In that case, keep MixFix OFF (otherwise you are literally injecting left into the right channel).

- If there is **no change** between RED and GREEN:
  - You are probably not listening at the right speaker, or the pattern didn’t load, or something is physically wrong.


### Pair 1 (I2S1): Slot2 + Slot3

#### Pattern 4 – I2S1 LEFT only (550 Hz)
Command: `DIAG A 4`
- Slot2 LED = WHITE
- **Speaker3 (slot2):** loud **550 Hz**
- **Speaker4 (slot3):** silent at the cone

#### Pattern 5 – I2S1 RIGHT only (1100 Hz)
Command: `DIAG A 5`
- Slot3 LED = WHITE
- **Speaker4 (slot3):** loud **1100 Hz**
- **Speaker3 (slot2):** silent at the cone

#### Pattern 6 – I2S1 L+R together (550 Hz + 1100 Hz)
Command: `DIAG A 6`
- Slot2 + Slot3 LEDs = WHITE
- **Speaker3:** only **550** at the cone
- **Speaker4:** only **1100** at the cone

#### Pattern 13 – PHASE-CANCEL I2S1 (slot2=+550, slot3=−550)
Command: `DIAG A 13`
- Supporting evidence for mono L+R summing (see Pattern 12 note).

#### Pattern 15 – MIXFIX TOGGLE (I2S1) — MOST DECISIVE
Command: `DIAG A 15`
- Slot2 LED = WHITE (tone)
- Slot3 LED alternates **RED ↔ GREEN**

Listen at **Speaker4 (slot3)** and use the same interpretation as Pattern 14.


### Full-system sanity & game reproduction

#### Pattern 7 – all four different tones
Command: `DIAG A 7`
- All LEDs WHITE
- Each speaker should have its own unique pitch (330/660/990/1320)

#### Patterns 8–11 – odd/common simulation
Commands: `DIAG A 8` ... `DIAG A 11`
- GREEN = odd (1600 Hz)
- RED = common (800 Hz)

Use these to reproduce the “odd sounds like it has odd+common” complaint with known tones.

---

## 5) Fast decision tree (recommended)

### Step 1 — Run the decisive toggle tests
- Run **DIAG 14** (I2S0)
- Run **DIAG 15** (I2S1)

If the RIGHT speaker gets quieter on GREEN → electrical mix.
If it gets louder on GREEN → acoustic bleed.

### Step 2 — If it’s electrical mixing
Enable MixFix during gameplay:
- `MIXFIX ON`

Or target a single bus if only one is affected:
- `MIXFIX BOTH 1 2000 1000` (I2S0 only)
- `MIXFIX BOTH 2 2000 1000` (I2S1 only)

### Step 3 — If it’s acoustic bleed
Keep MixFix OFF:
- `MIXFIX OFF`

Then mitigation is physical or mix/volume based:
- lower `MASTER_GAIN_DB` (Side)
- use `AUDIO_MITIGATION_MODE` trims (Master)
- add physical isolation/damping in the cabinet
