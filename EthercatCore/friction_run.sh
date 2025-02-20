#!/bin/bash
start_time=$(date +%s)

cd /home/user/release/friction_id_motion
./static_data_get
./dynamic_data_get
cd /home/user/release/friction_Id
python3 static_calculate.py
python3 dynamic_calculate.py

end_time=$(date +%s)
execution_time=$((end_time - start_time))
echo "전체 실행 시간: ${execution_time}초"