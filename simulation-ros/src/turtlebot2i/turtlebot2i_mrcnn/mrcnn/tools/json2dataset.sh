#！/bin/bash
str1="/home/enwhsaa/GetTrain/TrainSetDoor/json/"
str2=".json"
for((i=1;i<63;i++))
do 
str3=${i}
labelme_json_to_dataset ${str1}rgb_${str3}${str2}
done