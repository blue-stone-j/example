from datetime import datetime,timezone
import time

# Get current date and time
now = datetime.now()

# Print time in HH:MM:SS format
'''
%Y - year (e.g., 2025)
%m - month (01-12)
%d - day (01-31)
%H - hour (00-23)
%M - minute (00-59)
%S - second (00-59)
'''
print(now.strftime("%H:%M:%S"))
# Full date and time
print(now.strftime("%Y-%m-%d %H:%M:%S"))

# Seconds since epoch
print(time.time())  
# high-resolution timing
print(time.perf_counter())

# counter for performance measurement
start = time.time()
end = time.time()
duration = end - start
print(f"Duration: {duration:.3f} seconds")

start = time.perf_counter()
end = time.perf_counter()
print(f"Duration: {end - start:.3f} seconds")

start = datetime.now()
end = datetime.now()
duration = end - start
print(f"Duration: {duration.total_seconds():.3f} seconds")


timestamp = 1747204350012 / 1000
dt = datetime.fromtimestamp(timestamp, tz=timezone.utc)
print(dt.isoformat() + "Z")