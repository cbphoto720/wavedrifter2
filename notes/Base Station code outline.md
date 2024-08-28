## Init
- track num of drifters
## Startup
- connect to mission planner
- start RFM
- Broadcast Base UTC time (rfm 255 broadcast to all)

## Loop
- heartbeat
- report drifter health & location
- send drifter commands (recovery, shutdown, etc.)