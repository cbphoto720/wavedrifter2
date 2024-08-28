## Init
- [x]  track num of drifters
- [x]  buffer that tracks the last comm record for each drifter.
## Startup
- [x] connect to mission planner
- [ ] start RFM
- [ ] Broadcast Base UTC time (rfm 255 broadcast to all)

## Loop
- [x] heartbeat
- [x] report drifter health & location
- [ ] send drifter commands (recovery, shutdown, etc.)
- [ ] track last time drifter reached comms