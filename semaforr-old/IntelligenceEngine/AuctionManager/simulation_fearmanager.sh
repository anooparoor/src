#!/bin/bash

################################################################################
# run_stage_experiment
#
#  Runs a simulation for each configuration of the 2011-2012 auction task
#  experiment. Logs are saved in $LOG_DIR, defined below.
#
#  Usage:    ./run_stage_experiment
#
#            The current working directory must be the same as that of the
#            script (i.e., HRTeam/bin).
#
#  Authors:  Tuna A. Ozgelen
#            Eric Schneider (small tweaks)
#
#  01/12/2012 Initial git commit

### Path definitions

PROJ_DIR=`pwd | sed -e "s/bin$//"`
BIN_DIR=$PROJ_DIR/bin
CONFIG_DIR=$PROJ_DIR/etc/Current/Stage
LOG_DIR=$PROJ_DIR/logs

# Time and retry limits in the case of deadlocked runs

RUN_TIME_LIMIT=600  # in seconds (12 minutes)
RUN_RETRY_LIMIT=10

STAGE_FILE="$CONFIG_DIR/surveyor-3-allinone.cfg"
#STAGE_FILE="$CONFIG_DIR/surveyor-8-2rooms.cfg"

num_runs=5

for scenario in A B C D E
do

  sleep 3

  for mNum in 0 1 2 3 # mNum (method number: 0=random, 1=psi, 2=osi, 3=ssi)
  do

    method=

    if (( mNum == 0 ))
    then
      method="Random"
    fi

    if (( mNum == 1 ))
    then
      method="PSI"
    fi

    if (( mNum == 2 ))
    then
      method="OSI"
    fi

    if (( mNum == 3 ))
    then
      method="PSI"
    fi

    for RUN_NO in `seq 1 ${num_runs}`
    do

      REDO_RUN=""

      RUN_ATTEMPT=0

      while (( RUN_ATTEMPT < RUN_RETRY_LIMIT ))
      do

        let "RUN_ATTEMPT += 1"

        echo `date` ": Scenario:" $scenario ", RUN_NO:" $RUN_NO ", Attempt #" $RUN_ATTEMPT

        cs_log=SIM_CS_${method}_${scenario}_${RUN_NO}_${RUN_ATTEMPT}.log
        echo Log filename: ${cs_log}

        # start Central server
        echo "Starting Central server..."
        xterm -e $BIN_DIR/SkyGrid -l $LOG_DIR/${cs_log} -p 6667 &
        sleep 3

        # start player/stage
        echo "Starting Stage..."
        export LD_LIBRARY_PATH+=/usr/lib/gearbox
        xterm -e player $STAGE_FILE &
        sleep 3

        # Hack. A RobotController's config file (e.g., robot-stage1.conf)
        # specifies a map file that's assumed to be in the CWD. We cd to
        # the config directory, start the controller xterms then cd back
        # to the original CWD.
        EXEC_DIR=`pwd`
        cd $CONFIG_DIR

        # start robots
        for robot_num in {1..4} # 5 6 7 8
        do

          echo "Starting RobotController ${robot_num}..."
          rc_log=SIM_RC_robot-${robot_num}_${method}_${scenario}_${RUN_NO}_${RUN_ATTEMPT}.log
          echo "(cmd: xterm -e ./RobotController -d -f robot-stage${robot_num}.conf > $LOG_DIR/$rc_log &)"
          xterm -e "$EXEC_DIR/RobotController -w ${robot_num} -d -f robot-stage${robot_num}.conf > $LOG_DIR/$rc_log 2>&1" &
          sleep 3

        done

        #					cd $EXEC_DIR

        # start auction manager
        echo "Starting Auction Manager"

        cd $EXEC_DIR

        am_log=SIM_AM_${method}_${scenario}_${RUN_NO}_${RUN_ATTEMPT}.log
        echo "(cmd: xterm -e ./AuctionManager -t $mNum -f $(dirname $CONFIG_DIR)/AuctionManager/ARMS13_points_${scenario}.conf > $LOG_DIR/$am_log)"
        xterm -e "./AuctionManager -t ${mNum} -f $(dirname $CONFIG_DIR)/AuctionManager/ARMS13_points_${scenario}.conf > $LOG_DIR/$am_log 2>&1" &
        sleep 3

        init_pid=`pidof AuctionManager`
        sleep 1

        echo "Type 'r'<enter> to re-do this run in case of a collision deadlock"

        run_duration=0

        while [ `pidof AuctionManager` ]
        do
          sleep 1

          # Increment duration; we may need to time out
          let "run_duration += 1"

          # Read input
          read -t 0.1 REDO_RUN

          # Redo the run if max time is reached by simply setting
          # REDO_RUN=r manually
          if (( run_duration >= RUN_TIME_LIMIT ))
          then
            echo "Time limit exceeded"
            REDO_RUN=r
          fi

          # If user enters 'r' then we'll redo this run after killing
          # allof the services below.
          if [ "$REDO_RUN" == "r" ]
          then
            echo "Aborting run"
            kill -9 $(pidof AuctionManager)
            break
          fi

        done

        sleep 5

        echo "Run finished"

        echo "Stopping robot controllers"
        killall -9 RobotController

        echo "Stopping player"
        kill -9 $(pidof player)

        echo "Stopping Central server"
        kill -9 $(pidof SkyGrid)
        sleep 5

        # If we didn't get input that tells us to redo this run, break and
        # move on to the next one
        if [ ! $REDO_RUN ]
        then
          break
        fi

      done # End "redo" run

    done # End run number

  done # End mNum (method number: 0=random, 1=auction)

done # End scenario ['A'..'E']


