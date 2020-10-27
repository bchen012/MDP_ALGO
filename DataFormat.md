# Data Format

1. How to represent Map?
   Refer to briefing material: Map descriptor format.pdf

2. Format of sensor data
   e.g. 221301 ( each byte: how many blocks ahead
   six sensors, clockwise from top left corner)

3. Different modes
   EXPLORE ( android -> rpi -> algo, start of exploration )
   COMPUTE|221301 (rpi send sensor data to algo to compute next action)
   WAYPOINT|2|15 (rpi send waypoint coordinate to algo)
   MOVEMENT|LF (algo send next move to rpi)
   FASTEST (rpi send algo to compute fastest path)

4. Format of movement
   e.g. MOVEMENT|LRF
