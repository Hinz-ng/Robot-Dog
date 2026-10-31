# Belt-on baseline — command sheet

*Router: [TECHNICAL_REFERENCE.md](../TECHNICAL_REFERENCE.md). This sheet names MT6835;
MT6816 data are legacy. Check build/plant applicability in
[WORKING_MEMORY §24](../WORKING_MEMORY.md#24-unresolved-documentation-decisions) before use.*

Revised procedure (J01, then J03, same sitting)

Flash with pio run -e J01 -t upload (use -e J03 for J03). Any key aborts a running phase or ladder, so keep your hands off the keyboard while one runs. Codes in brackets are the BELT_DRIVE.md step names.

A. Before power-on
Step	Name	Action	Pass
A1	Hand-turn check	Power off. Turn the output by hand one full revolution each way	Smooth, no catch
A2	Slip marks	Pen line across pinion/shaft and across output pulley/hub. One mark on the side face of each idler roller	—
A3	Breakaway position marks	Fixed reference mark on the top plate beside the output pulley. Five marks on the pulley face at teeth 0, 22, 44, 66, 87 (of 108; spacing 22/22/22/21/21). Label them P1–P5. Also note which angle the clamp holes line up at	Marks reusable for every future breakaway check on this joint
B. Boot 1, output free (don’t reboot until section D)
Step	Name	Keys	Pass
B1	Boot banner check	Power on, open the monitor	MT6835 … crc_err=0 status=0 (weak=0 uv=0 ovs=0). Belt field: J01 10mm-9:1, J03 OFF
B2	Bus-voltage check	Meter on DC V, 20 V range, across the bus pads; compare with Vbus=	Within ±0.03 V; record both numbers
B3	Encoder link test	e	ENC PASS
B4	Alignment verify [B1]	V	≤ 8° elec and dir CW vs CW (last time: J01 2.42°, J03 2.73°). If > 8°, stop and send the line
B5	Electrical tripwire: phase 1 then resistance [B2]	1, wait for PASS, then 3. The rotor locks itself; don’t touch it	R: J01 0.2168–0.2302 Ω, J03 0.2238–0.2376 Ω
B6	Drag sweep, forward first [B3a]	5. About 50 s of free spin, hands clear of the belt	First-point speed ≥ 20 rad/s. If it aborts, stop and send the line
B7	Idler and slip check	Look	Each roller’s mark has moved; neither roller has slid along its shaft; both slip lines still aligned
B8	Drag sweep, reverse first [B3b]	-, then 5 within 0.8 s	Header says reverse first (if not, abort with any key and retry). Pooled drag_c 0.25–0.45 A each direction (predicted J01 ≈ 0.32/0.33, J03 ≈ 0.33/0.35). Ignore Ke
B9	Breakaway, 5 output positions [B4]	At each mark P1–P5: turn the output until the mark meets the reference, let go, wait 2 s, B; wait 2 s, b	10 M4 lines. Each: no NO MOTION, ≤ 0.60 A, travel ≤ 76.7 mrad (motor-side; 800 counts at 16-bit). On long pre-slide creep, aborted or a travel fail: re-settle and redo that position. Mean 0.20–0.45 A
C. Clamp (no key needed; nothing is running)

Turn the output to the clamp angle. Fit 2 × M3×10 countersunk screws into the top plate, perpendicular. Write down the pulley ID (and md5), belt ID, idler state and bearing-seat state. From here until unclamping, never use V.

D. Boot 1, clamped
Step	Name	Keys	Pass
D1	Backlash ladders ×2 [B6a]	w (~80 s, SW,… lines). When it finishes, w again. Don’t touch or re-clamp between them	Report ladder intercept/slope. G from B9 is conditional on free-output friction applying while clamped: only then use ≤ 29 µm pass, 29–59 marginal, > 59 slack for the accepted short-pulley plant. No G verdict for long-shaft J01 without clamped-friction evidence. If the skip detector stops a ladder, stop and send
D2	Ring set 1 (belt stiffness) [B6b]	2 rings with k, then 2 with K. Each ring: t → g (telemetry shows m=TORQUE(V)) → k or K → wait for CAPTURE done -- press d to dump → x → d. Press t before every ring	Label each dump boot 1 / k or K
E. Boot 2, still clamped
Step	Name	Keys	Pass
E1	Clamped reboot	Power-cycle → e → confirm stored MT6835 ZEA/dir usable → f (never F or V while clamped)	ENC PASS, alignment loaded
E2	Ring set 2 [B6b]	2 × k + 2 × K, same sequence as D2	Across all 8 rings: J01 and J03 f_d within 15% (J01 last time 65.0 Hz)
F. Close-out
Step	Name	Action	Pass
F1	Final slip check	Look at both slip lines	Still aligned
F2	Unclamp + hand turn	Remove the screws; one revolution by hand each way	Smooth, same as A1
