# Demo Narration Scripts

Two voice-over scripts, one per demo video. Each `[VISUAL: ...]` line tells you
what should be on screen for the line that follows. Sections are paced at
roughly 10–15 seconds; speed up or slow down to match the actual video.

---

## Drawing Demo (~60 seconds)

**[VISUAL: input image — the single-line drawing — fades up on screen]**

> This is our test input: a single continuous line, hand-drawn. We're going
> to turn it into something the bot can physically draw.

**[VISUAL: camera view of the whiteboard with the four corner AprilTags visible]**

> Four AprilTags define our coordinate frame. Specs — our perception node —
> detects all four every frame and computes a homography that maps every
> camera pixel into normalized board coordinates. The bot's tag is tracked
> the same way, giving us continuous pose at thirty hertz.

**[VISUAL: image_waypoints preview showing the extracted contour with the START marker on one end of the line]**

> To extract a path, we threshold the image, find the line's outline, then
> detect its two endpoints by looking for the sharpest U-turns in the
> contour — not the points farthest apart in space, because for an S-curve
> those wouldn't match. We slice the contour between those two endpoints
> and resample to give the controller evenly-spaced targets.

**[VISUAL: bot starts moving on the whiteboard; live pose overlay]**

> The waypoint list goes to a comms bridge, which streams 3-byte packets
> over Wi-Fi to the bot's ESP32 — type, heading, x, y. The bot runs a
> lookahead PID on its omnidirectional base. As soon as it gets within
> 5 cm of the current waypoint, our bridge advances and sends the next one
> immediately — so the path stays continuous, no stalling between targets.

**[VISUAL: time-lapse of the bot completing the drawing]**

> Every waypoint reach gets logged, the index advances, the marker traces.

**[VISUAL: clean shot of the finished drawing on the whiteboard]**

> Image, in. Drawing, out.

---

## Erasing Demo (~60 seconds)

**[VISUAL: whiteboard covered in scribbled ink]**

> Erasing is harder than drawing. The system has to find every mark, drive
> a path that physically reaches each one, and stay clear of the corner
> AprilTags — if the bot rolls over those, it loses its own coordinate
> frame.

**[VISUAL: bot tracing a serpentine pattern across the board with the eraser]**

> We run it as a three-stage state machine. Stage one is a hardcoded sweep
> — an alternating-row serpentine, twelve waypoints, inset six inches from
> every edge. The eraser arm is mounted four and a half inches above the
> bot's AprilTag, so we account for that offset when generating
> tag-frame targets.

**[VISUAL: HUD showing "state = detect", red dots over remaining ink spots]**

> Stage two: detection. We warp the board into a flat square, then run a
> LAB-channel local-background subtraction — heavy Gaussian blur as the
> background, anything that deviates is ink. This catches any marker
> color and ignores smooth glare.

**[VISUAL: zoom on the masking — corner ellipses, bot exclusion zone, surviving spots]**

> We mask out the corner AprilTags, the bot's chassis around its current
> tag pose, and any other tag in view. Surviving centroids get snapped to
> a 10×10 grid — using cell centers instead of raw pixel positions makes
> the waypoint list deterministic, so the comms node doesn't keep
> resetting its progress on every frame.

**[VISUAL: bot navigating directly to ink spots; eraser landing on each]**

> Each detected spot is shifted down by the eraser offset and clamped
> into the inset rectangle. The bot revisits each one and cleans up what
> the sweep missed.

**[VISUAL: a second sweep across the board]**

> Stage three is a second sweep for anything that re-emerged from
> smearing.

**[VISUAL: clean whiteboard]**

> End state: clean.

---

## Notes for delivery

- Cut to the IDE / preview image briefly when introducing the curvature
  endpoint detection — a still of the START marker sells the algorithm in
  one shot.
- For the erasing demo, the HUD line `state=sweep_1 → detect → sweep_2` is
  the clearest visual signal of the state machine — try to keep it on
  screen during stage transitions.
- If you only have time for 30 seconds total, drop the AprilTag/homography
  paragraph from the drawing script and the masking-detail paragraph from
  the erasing script.
