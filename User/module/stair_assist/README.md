# Stair assist module

`StairAssist` coordinates four caller-owned axis adapters: one front pivot, one
rear support, and left/right track drives. The module exposes a low-level
command interface; climb sequencing and command-source selection stay in the
application.

The command uses front pivot angle in rad, rear support extension in mm, and
track motor output speed in rad/s. Project adapters perform the mechanical
conversion between those physical units and motor units. Limits and feedback
timeout are required configuration, so unknown mechanism travel or screw lead
must not be guessed.

The cycle is `UpdateFeedback -> SetCommand -> PrepareOutput -> CommitOutput`.
`PrepareOutput` updates adapter command buffers only. A DM adapter's
`CommitCommand` transmits immediately; an RM adapter may stage output to its
shared command bus. The operation is not atomic across the four devices.
`Enable` may also transmit immediately for DM motors.
