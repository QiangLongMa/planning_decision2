    pidlocal=$(pgrep -f planning_main_node)
    if [ -n "$pidlocal" ]; then
        echo "Killing planning_main_node process with PID $pidlocal"
        kill -15 $pidlocal
    else
        echo "local_node4 process not found."
    fi