
import json
import time
data = {
    "distance": 4,
    "left_ticks" : 40,
    "right_ticks" : 39
}
time.sleep(10)
print(json.dumps(data))