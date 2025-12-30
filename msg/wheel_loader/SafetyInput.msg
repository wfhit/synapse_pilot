# Safety input message
# Reports safety-critical input states

uint64 timestamp		# time since system start (microseconds)

# Safety buttons
bool emergency_stop_pressed	# Emergency stop button pressed (latching)
bool deadman_active		# Dead man switch active (operator present)
bool safety_override_active	# Safety override switch (maintenance mode)

# Hardware states
bool external_estop_triggered	# External E-stop relay triggered
bool safety_relay_ok		# Safety relay circuit OK

# Additional safety inputs
bool remote_estop_triggered	# Remote/wireless E-stop
