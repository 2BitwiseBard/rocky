# Pebble cockpit guide

The cockpit is one running simulation of Pebble with a browser page on top: two camera streams, a state feed, and every control the terminal playground has. Start it with `./rocky.sh cockpit` and open http://127.0.0.1:8765 on this machine, or the tailnet address on a phone (see [Phone and remote](#phone-and-remote)). Every **?** next to a panel title opens this guide at the matching section.

## First five minutes

1. Open the **Drive** tab. The robot stands in the world named in the header chip. The status line under the video says what it is doing: the reflex state (NORMAL in green is good), the command you asked for and the command the gait actually got.
2. Tap **▲** twice. Each tap adds 15 mm/s forward; the status line shows the new command. Tap **STOP** (the pad's middle, or the red round button that stays on every tab) to stop.
3. Tap **wave**, then **say hello**. The robot waves and answers in chord-speak (it never speaks words). The sound plays in this browser when "audio here" is ticked in Make → Voice & sounds.
4. Tap **shove 40 N**. The robot braces or falls; if it falls, the righter tries to stand it up. **reset** puts it back at the spawn pose.
5. Open **Talk**, leave the mode on *Talk (regex intent, no model)* and type `walk forward 30 cm`. The regex brain answers instantly with no model loaded.
6. Open **World**, pick the preset `obstacle course`, press **load**, then tap the map somewhere clear: that sends a goto there.

If something looks stuck, check the **sim** chip in the header first: `live` is good, `paused` means the pause button is on, `stalled` or `no feed` means the server side is in trouble (see [Troubleshooting](#troubleshooting)).

## Driving

- **Pad and keys.** ▲ ▼ ◀ ▶ move, ↺ Q and ↻ E turn, STOP stops. On a computer the keyboard works whenever no text box has focus: arrows or `W` `A` `S` `D`, `Q` `E` to turn, `space` to stop, `G` to wave. Each tap adds 15 mm/s (or a small turn rate) to the current command.
- **The budget note.** Every command is fitted into the gait's envelope (`WaveGait.budget`): the gait can only move its feet so fast at the current period, duty and step height. When your command is bigger than the envelope, it is scaled down and the status line (and the overlay on a computer) says so in amber. The robot is not ignoring you; it is doing the most it can.
- **What the overlay shows** (on a computer, over the chase video): `cmd` is what you asked, `gait` is what the gait got after the guards and the budget; `feet` is one dot per leg, filled while the foot touches the ground; `probe` is the stance probe per leg (S / P / H / — with millimetres); `h` is the kinematic body height; `gyro`, trips and falls come from the reflex supervisor; `servo real` means the servo model is on. For six seconds after a guard refuses a command, the refusal shows in red (for example `blocked: void at 24 deg — clear to release`).
- **STOP** (the red round button) zeroes the velocity, blends out a gesture, cancels a goto and stops streaming to real legs. The pad's STOP and `space` do the first three.
- **Saying stop.** A typed or spoken line with `stop`, `halt`, `freeze`, `whoa` or `abort` in it stops the robot with every brain, and needs no wake word. With a model brain (Local, Multimodal, Claude) the stop runs before any model is asked and does not wait for a turn that is still running; the reply is `stopped (safe-stop).` When the line says more than stop, a question or something to remember (`why did you stop?`, `remember that the stop button is red`) goes to the model after the stop, and its answer follows the stop's reply; the model cannot move the robot in that turn. Anything else in the line (`walk forward 30 cm and then stop`) is not run, and the reply says so. A model brain hands `don't stop`, `non-stop`, `stop sign` and `bus stop` to the model; Talk stops on them.
- **What a stop cancels.** A stop from anywhere (a stop line, the STOP button or `space`, the console's `stop` in any case, the MCP stop tool) also refuses motion (`operator_stopped`) to every line you sent before it: the turn that is running, and a line still waiting for it, with every brain, Talk included. Send the line again to run it. A line you send after the stop moves as usual.
- **More driving** has the gesture and chord-word pickers, fixed shoves (20 / 40 / 60 N at the shell rim, 0.4 s), camera presets and an orbit. The camera is shared: every screen watching this cockpit sees the same view.

## Talking

The **Brain & chat** panel picks who answers what you type or say.

- **Talk** is a regex intent parser, no model: instant, predictable, knows walking, turning, gestures, words and gotos. Best for voice.
- **Local model** sends the chat to llama-swap with the robot's tools. The model does the tool calls.
- **Multimodal** is one vision model that both sees (the robot's eye) and acts.
- **Claude** uses the Anthropic API when a key is set. Without one, run `./rocky.sh chat` in a terminal: Claude Code then drives this same sim over MCP.

With a model brain, a move said relative to the robot (`forward 30 cm`, `back up 20 cm`, `half a meter to your left`) becomes one `move` call in the robot's own frame: forward is where the robot faces, whichever way it has turned. The cockpit reads the robot's position and heading and walks there as an ordinary goto, so every guard applies and it ends the same ways (arrived, cliff, stuck, blocked, timeout, stopped). A move is at most 1.5 m. Map coordinates (`go to 0.4, 0.2`) and remembered places still use goto.

### Which brain for what

- **lfm2.5-vl** for fast tool calls: about 0.3 s per command (0.2 to 0.5 s measured), tool calls included, and it is already resident on the GPU beside the everyday driver. Pick it as the brain when you want voice commands to feel immediate.
- **qwen3.6-35b-a3b** for conversation: the everyday driver, better at talk and multi-step requests, several seconds per turn.
- **gemma-4-26b-a4b** for vision (the vision role, or as the multimodal model): the best image descriptions in the fleet, but it does not fit beside the resident pair, so the first **look** swaps models and can take from seconds to over a minute.
- **qwen3.5-9b** is the default multimodal model: one model that sees and calls tools. Measured 2026-09-24: 7.2 GB on the GPU, 6.5 s from cold to its first answer, 0.2 to 1.5 s per spoken command. It runs alone, so using it unloads the resident pair. How to switch, and how to judge a new model: [Choosing a brain](#choosing-a-brain).

The panel warns when brain and vision are not the qwen3.6-35b-a3b + lfm2.5-vl pair that shares the GPU, because then every look may swap models. Quarantined models never appear in the lists. Each role shows its fallback chain; when a model fails, the reply says which fallback answered.

### Hands-free

Tick **hands-free** next to the mic and the page keeps the microphone open; the mic button turns green and shows 👂 (it is not a push-to-talk button while hands-free is on). A level meter cuts your speech into utterances (0.7 s of silence ends one, 12 s is the cap), and each utterance is transcribed by whisper (the fast base.en service answers in about 1 s; the large model takes 18 to 20 s). The voice gate below decides what happens with each transcript.

### The wake word

The **Wake word** card sets the robot's name (default **Pebble**) and a **voice gate**:

- **name required** (default). Say the name alone and the robot chirps (the `acknowledge` chord, from this browser or the server's speakers, wherever "audio here" sends its voice), the mic button glows purple and counts down 8 seconds, and the Wake word card says so too: the next thing you say goes straight to the robot. Or start the sentence with the name (`Pebble, walk forward 30 cm`): it is sent at once. A line without the name that would move the robot waits in the chat box; press Enter or send to run it. If you were typing in the chat box, your text is left alone and the heard line shows in the chat with its own **send as command** button. A line that moves nothing (`hello`, `what do you see`) is sent as plain chat, with the mic button and in hands-free alike. With a model brain (Local, Multimodal, Claude) any line could turn into a goto, so every line without the name waits.
- **open mic**. Every transcript is sent as a trusted command. Use it when you are alone in the room.

The name is one word of 3 to 24 letters. The page's match is deliberately strict, because the gate is what keeps TV chatter and other people from moving the robot:

- The name has to start the sentence: among the first three words for a name of five letters or more, the very first word for a shorter one. Filler words (`hey`, `ok`, `okay`, `so`, `um`, `uh`, `oh`, `yo`, `hi`) do not count, so `hey Pebble, stop` works. With a custom name, `walk to the <name>` does not open the gate.
- A name of five letters or more may be one letter off, the way whisper mishears it (`peble`, `pebbel`). A shorter name must be heard exactly: with a name like `Max`, `mix walk forward` does nothing.
- No prefixes: `pebblestone` is not `Pebble`, `rocket` is not `Rocky`.

What "trusted" means: the brains refuse to move the robot on a voice line unless the line was confirmed (your Enter), carried the wake word, or came through the gate above. The server has its own check (D052, `harness/intent.py`), which knows only the names *pebble* and *rocky* and whisper's usual guesses at them (`pebbles`, `peble`, `pebbly`, `rockie`, `rocket`), anywhere in the sentence, and the chat lets such a line move the robot whatever the page sent. So with the default names the server's check is the looser one: `walk to the pebbles` passes it. A custom name is enforced by this page alone: it sends a line that passes its match as trusted. The page shares the name with other screens (a screen with no name of its own picks it up), but the server's voice check does not use it.

## Choosing a brain

The cockpit gives each job to a model, one per **role**, and the **mode** says which of them answers what you type or say. Both are set in **Brain & chat**: on a phone, the **Talk** tab; on a computer, the Talk group of the sidebar. The role choices are saved and come back when the cockpit restarts; the mode is not saved.

The four roles:

- **brain**: a text model that reads your line and calls the robot's tools (walk, turn, move, goto, gestures, chords, memory). It answers in Local model mode, and it is the last model every other mode falls back to.
- **vision**: describes the eye's picture whenever something asks to **look**, and draws the box that **find** turns into a bearing and a distance. It works in every mode, Talk included.
- **multimodal**: one model that gets the eye's picture with every message and calls the tools itself. Only models whose llama-swap name or description says they can see (vision, multimodal or mmproj) are offered.
- **stt**: the whisper model name the mic's audio is sent with. Which whisper service answers is decided when the cockpit starts: the fast one on :8086 when it is up, the large one on :8082 otherwise.

The four modes:

- **Talk**: no model, a fixed phrase parser. Instant and predictable.
- **Local model**: the brain role answers. When it wants to see, it calls look, which goes to the vision role.
- **Multimodal**: the multimodal role sees and acts in one call, with no hand-off between two models.
- **Claude**: the Anthropic API when a key is set; otherwise run `./rocky.sh chat` in a terminal.

Each picker marks a model **● warm** (loaded: it answers at once) or **○ cold** (the first message loads it, which takes seconds, or longer for a big model). Quarantined models never appear. A text-only model is refused as vision or multimodal, and the refusal shows in red under the pickers. A model installed while the page is open shows up after a page reload.

From a shell on this machine, the same call the pickers make:

```bash
curl -s -X POST http://127.0.0.1:8765/api/brain -H 'Content-Type: application/json' \
  -d '{"mode": "multimodal", "multimodal_model": "qwen3.5-9b"}'
```

The other keys are `model` (the brain), `vision_model` and `stt_model`, or `mode` on its own. Refusals come back under `errors`. `GET /api/models` shows the current roles, every model llama-swap offers and each role's fallback chain.

**One registry.** Every model mode (Local model, Multimodal, Claude) and Claude Code over MCP get their tools from one list, `harness/capabilities.py` (D056). Each tool is written there once: its name, its arguments, the text the model reads, whether a spoken line needs the wake word for it, and what it needs to exist (the eye, the memory, this cockpit). The gesture and chord-word lists inside the tools are this cockpit's current ones, so a gesture saved in the studio or a word saved in the chord designer can be called by name on the next message, and the MCP server's list follows them too, with no restart. Talk mode has no model and does not use the list. `docs/TOOLS.md` shows every tool, which ones need the wake word, and the envelope numbers the descriptions quote (a move is at most 1.5 m; a goto target should stay within ~1.5 m, which is what its 40 s covers, and one farther than 3 m is refused).

### When a model fails

Each role has a fallback chain, shown under the pickers (for example multimodal: qwen3.5-9b → gemma-4-26b-a4b). A model that times out (90 s), returns an error or answers nothing is skipped, and the next one tries. The reply then starts with a note in brackets that names the model that answered and why the earlier ones did not, and the chat shows each skip as a ↪ line. When the multimodal chain runs out, the text brain answers without the picture (it can still call look). When no model answers, the Talk parser does, unless tools already ran: then the reply lists what ran, and nothing is done twice. When llama-swap is down, the pickers say so and only Talk works. A fallback can mean loading another model, so that reply can take much longer than usual.

### What fits on the GPU

The card has 16 GB. Only the everyday pair stays loaded together: qwen3.6-35b-a3b as the brain and lfm2.5-vl as the eye (llama-swap's `resident` group). Every other chat model runs alone: loading it unloads the pair, and loading the pair again unloads it. The 9B, the 12B and the 35B never fit together.

- A multimodal model is one model and one load: nothing swaps while it works.
- A brain and a vision model that are not the resident pair swap on every look, which takes seconds to minutes each time. The amber note under the pickers warns about it.
- A new model never goes in the `resident` group. llama-swap does not check memory for a group, so a third member makes the next load fail with an out-of-memory error.

### Adding and measuring a model

A downloaded model goes in with `./rocky.sh brain-install` and is measured with `./rocky.sh brain-bench --models ID`. The steps and the table of measured models are in `docs/BRAIN_MODELS_2026-09-24.md`. Two things to know here: brain-install restarts llama-swap, so every loaded model unloads and this cockpit's next answer starts cold; and before each model's run the bench unloads every loaded GPU model, the one it measures included, so the load it times is cold.

**Leave this cockpit alone while brain-bench runs.** The bench starts its own cockpit on :8792 and never sends anything to this one, but the two share the GPU. Do not type or talk to this cockpit (turn **hands-free** off), and do not ask it to look or find. Any request here that reaches a model loads that model and evicts the one being measured, and the bench's next line loads it back. That repeated swapping is what this GPU punishes (it is the pattern behind the 2026-09-01 lockout), and it spoils the bench's VRAM and latency numbers.

### Reading a bench row

The bench prints one row per model. What each column tells you:

- **`vram_mib`**: GPU memory the model took once warm, out of 16,384. It decides what the model can run beside.
- **`first_answer_s`**: loading from cold plus the first reply. It is what you wait after switching to the model, or when a fallback or a look has to load it.
- **`decode_tps`**: generation speed in tokens per second. It matters for descriptions and longer replies; a tool call is short.
- **`cmd_ok`** (out of 20): spoken-style lines that got an acceptable tool with sensible arguments (`walk forward thirty centimeters` has to become about 0.3 m ahead, not 30: a `move` of 0.3, or a goto 0.3 m in front of the robot). The main number for the brain and multimodal roles.
- **`stop_missed`**: stop lines that did not stop the robot. It must be 0: one miss rules the model out for voice driving.
- **`unsafe`**: motion nobody asked for. It must be 0 too.
- **latency** median and p95 (s): the wall time of each command's chat turn, typical and worst case. It includes the motion itself: a gesture runs to its end and a walk until the robot arrives before the turn finishes, so a model that gets "wave hello" exactly right still shows several seconds here. Do not compare it with the tool-call times below.
- **first action** median and p95 (s) (`1st act s` in the printed table, median only): the time from sending the line to the first thing the sim shows (a stop, a chord, a gesture or a walk starting, a look). It counts only the lines that changed something in the sim, and it is polled ten times a second, so it can read up to 0.1 s late. A cockpit from 2026-09-25 on also numbers its events, so a repeat of the last event (one more stop after twelve) counts too. This is the model's decision time, and the number to compare with the hand measurements: lfm2.5-vl made tool calls in 0.2 to 0.5 s per command, qwen3.5-9b in 0.2 to 1.5 s.
- **describe**: whether the model mentions the ball when it is in view (and not when it is not), and on the correct side. For the multimodal and vision roles.
- **vision** (with `--vision`): how often find's box saw the object, and its median bearing and distance errors. For the vision role.

The bench runs on clean sim pictures, so its vision numbers are a best case for the real camera.

## Making gestures

The **Gesture studio** (Make tab) builds keyframe gestures, stored as JSON in `gait/gestures/`.

- **Pose and snap.** Move the sliders (body x / y / z, yaw, corner drops, one leg raised as an arm, a claw): the robot blends to the pose and holds it live. Set the frame time, press **snap frame**, repeat. **remove frame** deletes the frame at the current time; **new** starts over; **release pose** lets go of the held pose.
- **Scrub** moves through the gesture without playing it; **▶ play** runs it in physics; **save** writes it. Click a frame in the list to load it back into the sliders.
- **Reach and solve.** Put the arm leg's hand at x / y / z (millimetres, in the stance's ground frame; the floor is at z = −h) and press **solve**: it searches body lean, height, yaw and the arm joints that reach the target while the centre of mass stays inside the planted feet, then fills the sliders. Snap to keep it.
- **Teach by demonstration.** **record pose stream** samples the joint targets at 20 Hz while you pose, play a gesture or back-drive the real legs (robot → sim); stop turns the stream into keyframes. Check the fit residual it reports: above 5 mm the body pose is not trustworthy.
- **load draft** opens the last gesture a brain composed with `compose_gesture`.

### The feasibility verdict

Every change is judged by `gait/pebble_feasibility.py`, the same judge the gestures, the brains and the gait go through. The verdict box is green for PASS, amber for PASS with warnings, red for FAIL. A FAIL is not played or saved unless you tick **force**. The codes:

- `LIMIT_YAW`, `LIMIT_HIP`, `LIMIT_KNEE`: a joint outside its soft range minus a 2° guard band. `LIMIT_CLAW`: the claw outside 0 to 55°.
- `NAN`: a target the leg cannot reach at all.
- `SPEED_LOADED`, `SPEED_FREE`, `SPEED_HARD`: joint speed over the servo's budget: 3.0 rad/s on a loaded leg, 4.0 on a free leg, never past the no-load 4.7. `SPEED_CLAW`: the claw over 8 rad/s. Fix it by giving the frame more time.
- `JUMP`: a step of more than 2° between samples, usually at the start or end of the gesture (it does not start from, or return to, the standing pose).
- `SUPPORT`: fewer than three feet on the ground. `MARGIN`: the centre of mass within 10 mm of the support polygon's edge (`MARGIN_WARN` under 25 mm). `MARGIN_GAIT` (a warning) is the gait's version: the centre of mass within 10 mm of the stance polygon.
- `SELF_CONTACT`: the robot touches itself by more than 1 mm.
- `SLIP` / `SLIP_WARN`: the body accelerates harder than the feet can grip.
- `THERMAL` (warning): a joint spends a long time near its speed limit. `THERMAL_LOAD` / `THERMAL_LOAD_WARN` come from a physics playback (`sim/audit_gestures.py`), not from the studio.
- `LOOP_WRAP`: a looping gesture whose end does not meet its start. `REACH`: a reach target that cannot be solved. `SPEC`: a malformed gesture file.

## Gait lab and realism

The footfall diagram draws one row per leg, filled while the foot is down; the white tick is the gait phase. The sliders set period T, duty, body height, stance radius, step height and the gyro trip threshold, live and sim-only. **check** runs the feasibility judge on the gait at the current command and at the envelope corners. Presets load from and save to `gait/gaits/` (`default` is params.yaml; `legacy_d050` has no envelope under the D052 budget).

The **servo model** (on by default) makes the sim's joints behave like ST3215 servos on a bus: a 50 Hz hold, latency, a slew rate that drops under load, 4096-count quantisation. Turning it off gives the ideal actuator, which flatters every motion.

## Sounds

Pebble speaks in chord-speak: short chords, never words. The row of word buttons plays any word in the lexicon on the robot. **audio here** plays the robot's words in this browser (right for a phone on the tailnet); unticked, the server's speakers play them.

The **chord designer** builds new words. A word is one to four syllables; each syllable is a stack of just-intonation ratios over a low root (`1 1.5 2`, or `1 7/6 7/4`), with duration, gap, loudness, roughness, breath, pitch bend, detune and wobble. Load a canon word to see how it is built, change it, **▶ hear**, then **save word** under a new name: it joins the lexicon for say, the brains and the MCP tool.

## Worlds and guards

The **World & map** panel loads presets (flat, room, cliff, obstacle course, rubble field, rough terrain, stairs, slope 8 deg, icy floor), adds objects at x / y, sets friction (0.8 is the foot pad's value) and slope, drops a rough patch ahead, builds a random course from a seed, and saves or loads your own worlds. The map shows the robot, the objects, lidar hits in blue, the goto target in cyan and a latched void as a red wedge. Tapping the map sends a goto. The lidar sees only what stands above its plane (about 0.18 m): a goto into low boxes or rubble ends `stuck`, into tall ones `blocked`.

The header chips that need you:

- **VOID N°** (red): the always-on void guard found no floor under a planted foot, at that bearing. Walking or a goto toward that side is refused until you press its **clear** button (or type `clear`). Move away from the edge first.
- **LATCHED** (red): three gyro trips in five seconds latched a safe-stop; velocity commands are ignored until **clear**. Clearing does not resume the old walk.
- **locomotion held** (amber): the sim is streaming to real legs (sim → robot) and the real feet report no contacts yet. Walking and teleop are disabled; gestures are allowed after the soft entry. It clears when the real feet touch down, or when you set the mirror back to off.
- **servo heat N%** (amber, red when tripped): the hottest leg joint's share of its thermal budget. Sustained load above 0.65 × stall heats it; about 3 minutes at 0.85 trips it. The sim only accounts heat (it does not weaken the joint), but a tripped joint refuses sim → robot. It clears by resting: stop, stand still, let it cool.
- **robot ≠ ckpt** or **robot ?** on the fingerprint chip: see [Model and fingerprint](#model-and-fingerprint).

## Memory and awareness

Pebble keeps a memory per world: what it found, what you told it, which guards fired, what changed around it. The **Memory & awareness** panel (World tab) shows it, and the map draws every remembered thing as a pink ring.

- **What is remembered.** Sightings from **find** (the eye's box projected onto the floor), your notes and pins, guard events (a void, a latched safe-stop, servo heat, a bus fault), new obstacles and scans from the lidar, the chords it said on its own, and hints from a **look** description that names a thing with a direction and a distance. Each thing carries its position in map metres, how sure it is (confidence 0 to 1) and where that came from.
- **Where it is saved.** `sim/out/memory/<world>.json`: a preset under its own name (`obstacle_course.json`), an edited or random world under its name plus a hash of its spec. It is written a moment after every change and loaded again when the world comes back, also after a restart. **clear memory** keeps the old file as `.json.bak`. At most 500 observations per world; the least useful go first. `--no-memory` keeps it in RAM only; `ROCKY_MEMORY_DIR` moves it.
- **The map.** A solid ring is confident enough to walk to (0.4 or more); a faint dashed one is a vague hint; a grey one is stale. The glyph above it says where it came from: an eye for find, a pin for you, `?` for a look, a flag for `start` (the spawn). Click a ring for **go back** or **forget**. Right-click the map (long-press on a phone) and type a name to pin that name at that point.
- **The panel.** The situation line, **find** (look, turn or walk toward it, look again: up to about 3 minutes, and it walks, so every goto guard applies and STOP cancels it), **remember** for a note, the list of remembered things with their position, age, confidence and source (each with go back and forget), the last 10 observations (refreshed every 5 s while the panel is open), and **clear memory**. On a phone, the Drive tab shows the situation as one line under the status line; tap it to open this panel.
- **By voice or chat.** `where is the ball` · `go back to the box` · `remember that the charger is by the door` · `the charger is here` (pins the robot's own position) · `what do you remember about the ramp` · `forget the ball`. go back and find move the robot, so a spoken one needs the wake word like any motion; `forget everything` by voice needs it too.
- **go back** walks in gotos of at most 1.2 m (up to four), each with every guard, and stops about 0.35 m short plus half the object's size (right on a place you pinned with "is here"). It walks to where the thing was remembered and does not look again: the reply says so.

**Stale** means older than 10 minutes, or seen before the last reset or world load (the world puts its objects back at their spawn). go back refuses a thing from before the reset unless you pinned it yourself; find looks for it again. Places pinned at the robot's position (`the charger is here`, `start`) never go stale.

A reset, a world load or an edit also forgets the last look and find, so the next situation line says `eye: no description yet`. A look or find that is still running when that happens is not remembered: its answer describes the scene before the change, and the reply says so.

The awareness settings:

- **situation** (0 to 60 s, 0 = off; the default is 20 s, `--awareness-s` on the command line): how often the robot composes its situation line while it stands idle: pose, guards, the nearest lidar returns, the nearest remembered things, servo heat, the bus. A chat turn always composes a fresh one; **now** composes one at once.
- **curious**: at most one unprompted look a minute when the lidar scene changed. It is a vision-model call, so it may load a model on the GPU. Off by default.
- **reactions**: a chord when a guard latches or something new appears within 0.5 m, at most once per 30 s.

How far to trust a position: a find sighting is the vision model's box projected onto the floor through the eye's measured pose. The vision bench measured the near edge within 3 cm (median) in clean sim renders out to 1.4 m; on the real camera it is unmeasured. A sighting farther than 2 m is kept as vague. A look description is a guess (0.30 m median error measured), a hint for the map only, never a place to walk to.

## RL panel

- **Righter** is the policy that runs while the robot is FALLEN. Pick one and **load**, or **off** for the bare supervisor. To compare two, shove 40 N with each on the same world. `recover1` is the shipped righter. Honest numbers (docs/RL_GUIDE.md §4): with the supervisor around it the system stands 20 of 20 falls, but the policy alone earns no clean handoff on the current (D052) model; without any righter the supervisor stands 11 of 20.
- **Walking** swaps the analytic wave gait for a PPO residual policy (±0.25 rad on the gait's targets, 50 Hz). The analytic gait still wins on a clean floor.
- **stall s** and **deadline** tune the FALLEN handling. Stall: when the righter has not improved the robot's best tilt for this many seconds, the supervisor stops it and runs its planted ramp (which rights the robot from its back, the case no checkpoint solves). Deadline: the most seconds the righter gets before the ramp runs anyway.
- The table lists the runs on disk: `obs` is the observation version the checkpoint expects, `rew` the reward version, `rate` its action rate limit, `steps` training steps in millions, `ret` the final training return, `robot` the fingerprint it was trained on (green when it matches this robot), `eval` the last evaluation note.
- **Legacy checkpoints**: everything trained before D052 (obs v1, no fingerprint) is marked `legacy obs` and replayed as it was trained, but its numbers were earned on different physics. Re-evaluate before trusting one. Retrain with `./rocky.sh train-recover NAME`; audit with `python sim/audit_righter.py runs/NAME/latest.pt`.

## Model and fingerprint

What the sim believes the robot is: servo stall and continuous torque, speed budgets, soft limits, the gait and its envelope, body geometry, and the studio's slider ranges, all from `cad/params.yaml` through `gait/rocky_model.py`. The **fingerprint** is a short hash of those numbers; every checkpoint is stamped with the fingerprint it was trained on. The header chip turns amber (`?`) when a loaded checkpoint has no fingerprint (pre-D052) and red (`≠ ckpt`) when it was trained on another robot.

## Recordings

**⏺ record** in the header records the chase camera plus every command. Stopping asks for a name and writes `clip.mp4`, `clip.gif` and `run.json` (the world and the commands). In Robot → Recordings, **▶ replay** reloads that world and re-runs the commands; the file links open the clips.

## Hardware bring-up

The Hardware panel talks to the Feetech servo bus beside the sim. No real servo has been on the bus yet: everything below is tested on the byte-level `mock` port only. The order for the first real servos (backlog B32):

1. **connect** the adapter's port (or `mock` to rehearse), then **scan bus**. Every leg whose three servos answer becomes a real leg.
2. **set id**: with one servo on the bus at a time, write its ID.
3. **center here**: torque off on that joint, hold it at the jig pose (`bench/BENCH_RUNBOOK.md` §5), click. It writes `bench/calibration.yaml`.
4. **dir test**: jog each joint a few degrees; if it moves the wrong way, **dir −** flips its sign.
5. **robot → sim**: move a real leg by hand and confirm the sim leg follows the same way (the IK frame is right).
6. **sim → robot** with the stream speed at 200 c/s for the first stance. It needs the sim at a planted standstill, at 1× speed, not paused.

**Soft entry** is what sim → robot does first: each real leg's goal is parked where the leg already is, torque comes on at 40 %, the stream is capped at 200 counts/s, the goal blends to the sim's target, and only then is it released to full torque. Locomotion stays held until the real feet report contacts.

**LIMP** turns torque off on every servo at once: the legs go slack. Use it whenever something looks wrong. **apply limits…** burns the soft limits into each servo's EEPROM angle limits, with a dry-run table first. A joint that trips (heat, fault, silence) cuts its whole leg; **re-arm** in its row is the only way back.

## Phone and remote

Publish the cockpit to your tailnet with `./rocky.sh tailnet`, then open **https://ai-hub.tail54f481.ts.net:9445** on the phone (tailnet only; `./rocky.sh tailnet off` removes it). It must be https: browsers only allow the microphone on https or on localhost.

On a phone the page is one tab at a time with a bar at the bottom: **Drive**, **Talk**, **Make**, **World**, **Robot**. The header is one line you can swipe sideways; it starts with the title, then a red VOID or LATCHED chip when one is up, then **? guide**. When a guard blocks the robot, the Drive tab's status line says so and a **clear the guard** button appears right under it. The red **STOP** button floats on every tab; the page leaves room under the last control so STOP never sits on top of it once you scroll to the end. The console under the pad is a three-line ticker; tap it to expand. Video only streams while the Drive tab is showing, so the other tabs cost no bandwidth. Every **?** opens this guide in the Robot tab; a **← back** button at the top of the guide returns to the tab you came from.

- **remote** (Drive tab) turns this screen into a remote control: both video streams are closed (no bandwidth at all) and the pad, status line and quick buttons get bigger. Watch the robot on the laptop instead.
- **screens: lead / follow.** Set the phone to **lead** and the laptop to **follow**: when you change tab or open a panel on the phone, the laptop opens the same one. The camera view is already shared by the server, so the laptop's video follows camera changes by itself. **independent** turns this off. Each device keeps a random id in its browser; the choice is remembered per device.

On a computer the sidebar shows the same five groups as tabs across its top. **pin** shows every group at once, as one long accordion (the old sidebar).

## Troubleshooting

- **Frozen video.** The page reconnects a stream after 3 s without a new frame. If it stays frozen, look at the **sim** chip: `stalled` or `no feed` means the server is in trouble, `DEAD` means the sim thread stopped (restart with `./rocky.sh cockpit`). On a phone, video only runs on the Drive tab and never in remote mode, by design. Reloading the page always reconnects.
- **"recording too short".** The mic got less than about half a second of audio. Hold the 🎤 button while you speak, or tap once to start and tap again to stop; phones sometimes lose a long press to a scroll.
- **"nothing heard".** Whisper returned no words. Speak closer to the phone and a little longer. If it keeps happening, check that the whisper service is up (`systemctl --user status whisper-fast` for the fast one on :8086, `whisper-server` on :8082).
- **Tailnet: http vs https.** `http://…:9445` does not work and a plain-http address gives no microphone. Use `https://ai-hub.tail54f481.ts.net:9445`. If the page does not load at all, check `tailscale status` on both devices.
- **Two cockpit tabs in one browser freeze.** A browser keeps at most six connections open to one address, and each cockpit tab holds three for good (the state feed and two video streams; four when following). Two full tabs in the same browser use them all and everything else waits. Use remote mode in one of them, or two devices. Lead / follow also needs two devices or two different browsers: the device id is kept per browser, so two tabs of one browser are the same device.
- **"port 8765 already in use".** Another cockpit is running (maybe started in the background). Stop it with `./rocky.sh cockpit-stop`. `rocky.sh` runs one cockpit at a time (`./rocky.sh cockpit` stops any running one first); a second one needs `python sim/cockpit.py --port N` started by hand, and `ROCKY_COCKPIT_PORT=N ./rocky.sh tailnet` points the phone at it.
- **"no shared view" or "the guide is not served".** The shared view (lead / follow) and this guide come from `sim/cockpit_shared.py`; a cockpit whose `make_app` does not mount its routes answers 404 for `/api/ui` and `/api/guide`. Everything else works; the screens stay independent.
- **A command does nothing.** Look for a red line in the status line or overlay: a guard refused it and says why. Check the header for VOID, LATCHED or locomotion held.
- **Voice moved nothing.** The line was heard without the name, or the name was not at the start. Say the name first, press Enter on the transcript, or switch the gate to open mic.
