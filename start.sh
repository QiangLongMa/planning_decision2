#! /bin/bash

pid1=$(pgrep -f hmi)
if [ -n "$pid1" ]; then
    echo "Killing hmi process with PID $pid1"
    kill -15 $pid1
else
    echo "hmi process not found."
fi

pid2=$(pgrep -f qt)
if [ -n "$pid2" ]; then
    echo "Killing main_window process with PID $pid2"
    kill -15 "$pid2"
else
    echo "main_window process not found."
fi