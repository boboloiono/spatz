# Vector Matrix Engine

VME is the matrix execution IP used by Spatz. The initial extracted
configuration targets FP16 operands and FP32 accumulation with a 16 x 16
logical tile, an 8 x 8 compute edge, and four resident FP32 accumulator tiles.

## Parameter model

VME separates parameters according to what they are allowed to change. Values
that affect commands or packed interfaces are constants in `vme_pkg`; only
implementation choices are module parameters.

| Class | Parameters | Baseline | Where configured |
| --- | --- | ---: | --- |
| Architectural command contract | logical tile count, maximum K steps | 16, 4 | `vme_pkg.sv` |
| Host integration contract | in-flight IDs, VLEN, vector-register count, VRF word width | 8, 512, 32, 256 bits | `vme_pkg.sv` and checked by the host adapter |
| Propagated structure | tile edge, accumulator-word width/bytes, compute edge, spatial beats, maximum VRF-group words, FPU pipeline registers, optional FP16/FP32 FMA datapaths | 16, 32/4, 8, 4, 16, 4, disabled/disabled | adapter and `vme` module parameters |
| Derived | VRF words, accumulator banks/depth, spatial beats, address and mask widths | computed | never overridden independently |

For example, changing `ComputeEdge` changes the number of PEs and causes
spatial-beat count and accumulator depth to be recomputed locally. `TileEdge`,
`AccWordWidth`, and `AccWordBytes` enter at the host adapter and are propagated
through `vme` to the scheduler, mover, compute array, accumulator, and tile LSU.
The package defaults still size packed host-interface types, so a non-default
integration must provide a matching package and adapter configuration.

The physical accumulator word has a fixed elaboration-time width. Its write
enable has one lane per byte, while `msetmtype.tew` dynamically groups those
lanes into one, two, or four-byte elements. For a 32-bit accumulator word the
runtime number of element masks is therefore `AccWordWidth / TEW`: four for
TEW8, two for TEW16, and one for TEW32. The SystemVerilog port remains four
byte enables wide because a signal width cannot change at runtime.

`MaxVregGroupWords` sizes the mover interfaces and represents
`MaxLMUL * VLEN / VRFWordWidth`. The current instruction's word count remains
runtime state derived from the LMUL selected by `msetmtype`.

`src/vme_pkg.sv` owns the command, completion, VRF, and tile-memory types.
`src/vme.sv` does not import
`spatz_pkg` or `rvv_pkg`. The adapter at
`hw/ip/spatz/src/ope/spatz_ope.sv` translates decoded Spatz requests into the
smaller VME command and converts the completion and tile-transfer channels
back to Spatz types. Elaboration-time checks reject mismatched architectural
or integration contracts before simulation.

## Integration boundary

The host processor owns ISA decode, vector-register dependencies, and
architectural retirement. VME owns matrix execution, resident tile state,
unit issue, and local resource hazards. `compute_array.sv` is a distributed PE
array: every PE contains its FPU/SDOTP datapath and local accumulator, keeping
the feedback path local as `accumulator -> FPU addend -> FPU result ->
accumulator`. There is no centralized tile-register-file wrapper or
accumulator broadcast back into the compute array.

`mover.sv` owns tile-to-vector and vector-to-tile request lifetime,
word sequencing, VRF response buffering, and retirement. `tile_lsu.sv`
maps generic tile-word requests to the distributed accumulator ports. TCDM,
AXI, cache, and virtual-memory protocols remain in the host-side adapter.

Accumulator bank steering and per-PE lazy-zero metadata are owned by
`compute_array.sv`, beside the distributed accumulator state. The mover
crosses the compute-array boundary with request metadata and one VRF word.
Mover and tile-LSU writes decode directly into bank-local enables, so partial
updates do not read and rewrite an entire accumulator slice. The expanded
per-PE accumulator buses therefore do not pass through `vme.sv`.

Compute, move, and control operations enter independent `stream_fifo`
instances. `tile_scoreboard.sv` records logical-tile
readers and writers at dispatch, attaches RAW/WAR/WAW dependency IDs to each
queue entry, and releases a MAC tile only after its final FPU result reaches
the local accumulator. Independent tiles can therefore issue to different
units in the same cycle. The host `matrix_enable` signal remains responsible
for architectural and vector-register dependencies.

For a future Ara integration, keep host-specific decode, VRF protocol
conversion, and memory-system protocol conversion in an Ara adapter. The VME
side should use decoded commands, tagged completions, VME-owned VRF channels,
and a generic tile-memory request/response interface. It must not contain a
second architectural decoder or architectural vector-register scoreboard.

## RTL structure

`vme.sv` is only the extension boundary and scheduler. Implementation blocks
are named after their circuits: `compute_array`, `accumulator_tile`, `mover`,
`tile_lsu`, and `tile_scoreboard`. Internal RTL uses explicit combinational
wiring rather than SystemVerilog functions. Wide control fanout inside the
compute array is split into one branch per PE row; with the baseline 8 x 8
array, a root drives eight row branches and each branch drives eight PEs. The
request queues and FPU pipeline registers form the ready/valid boundaries, so
no combinational ready-to-valid feedback crosses units.

## Compute pipeline tags

The arithmetic result produced by an FMA or SDOTP lane does not retain the
identity of the command that generated it. `compute_array.sv` therefore shifts
one shared `pipe_tag` alongside the complete PE array. This is a metadata
pipeline, not an additional arithmetic-data pipeline and not one copy per PE.

The baseline four-stage compute pipeline consists of one shared
operand/addend register followed by three distributed registers inside each
FMA or SDOTP datapath. The tag has the same total depth:

```text
arithmetic: operand register -> FPU stage 1 -> FPU stage 2 -> FPU stage 3 -> result
metadata:   tag stage 0     -> tag stage 1 -> tag stage 2 -> tag stage 3
```

Each tag carries the following state for one spatial beat:

| Field | Purpose |
| --- | --- |
| `valid` | Marks an occupied compute-pipeline stage. |
| `completion` | Marks the final beat of the matrix instruction. |
| `id` | Identifies the instruction completed by the result. |
| `tile` | Selects the destination logical accumulator tile. |
| `beat` | Selects the destination spatial beat within that tile. |
| `mask` | Carries the byte lanes selected by the command's runtime TEW. |
| `lane_active` | Masks inactive PE lanes for partial M or N dimensions. |

At the final stage, `tile` and `beat` select the local-accumulator write
address, while `lane_active` controls which PE results are written. A matching
result tag also enables direct accumulator forwarding to the next MAC on the
same tile beat. All valid tags are reduced into `pipe_tile_busy_o`, which
prevents the mover, tile LSU, and incompatible compute accesses from touching
an in-flight tile beat. The scoreboard is notified with `id` only when a tag
marked `completion` reaches the result stage.

Keeping this identity in an explicit shared pipeline allows bubbles,
back-to-back operations on different tiles, partial shapes, and result
forwarding without deriving destination state from a timing counter.

## Standalone smoke test

The testbench drives only the VME-owned command, completion, VRF, and tile
interfaces. It writes and reads an accumulator row, executes a tile-zero
command, checks the completion ID, and verifies that the row reads back as
zero. From this directory, run:

```sh
mkdir -p build
cd build
vlib work
bender -d .. script vsim -t vme_test --vlog-arg="-sv -work work" > compile.tcl
vsim -c -do 'source compile.tcl; vsim tb_vme; run -all; quit -f'
```

The simulator environment, including its license, must already be configured.
The testbench top is `tb_vme` in `test/tb_vme.sv`.

## Extraction sequence

Completed:

1. Extract the implementation while keeping the Spatz integration regression
   green.
2. Replace `spatz_req_t` with the VME-owned `vme_req_t` command.
3. Replace direct Spatz response, VRF, and tile-transfer types with VME-owned
   types and explicit adapter conversions.
4. Move the `ope_fma`, `ope_sdotp`, and `accumulator_tile` implementation and
   their arithmetic helpers under this IP.
5. Add a standalone VME testbench independent of the Spatz cluster.
6. Build `compute_array.sv` with an accumulator local to every PE.
7. Isolate tile/vector sequencing in `mover.sv`.
8. Isolate generic tile-word packing and accumulator access in `tile_lsu.sv`.
9. Add per-unit operand queues and a VME-local logical-tile scoreboard.
10. Isolate MAC beat sequencing and VRF lookahead in `mac_scheduler.sv`.
11. Localize pipeline tags, forwarding, lazy-zero state, mover steering, and
    tile-LSU accumulator wiring in `compute_array.sv`.
12. Replace partial-slice read/modify/write paths with bank-local write
    enables, refine lazy-zero state to each PE word, and pack tile reads per
    element.

Next:

1. Connect the host memory adapter directly to the generic `tile_lsu`
   tile-word channel and remove the remaining Spatz-named integration signals.
