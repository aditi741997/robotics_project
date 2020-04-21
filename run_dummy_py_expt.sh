for lim in 1200 1800 2400
do
    for freq in 12 30 40 50 60 70 100
    do
        p="Tpub_$lim$freq.out"
        s="Tsub_$lim$freq.out"
        echo "a"
        taskset -c 5 python src/rbx1/rbx1_vision/src/rbx1_vision/test_publisher.py $freq > $p &
        taskset -c 6 python src/rbx1/rbx1_vision/src/rbx1_vision/test_subscriber.py $lim > $s &
        sleep 100s
        for pname in test_subscriber test_publisher
        do
            echo "Killing_$pname"
            kill -INT $(ps -ef | grep $pname | grep -v grep | awk '{print $2}') #| xargs kill -15
            sleep 1s
        done
        sleep 5s
    done
done
