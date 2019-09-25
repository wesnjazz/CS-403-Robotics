# Runs the roger simulator and multiple roger clients in the arena environment

ENVIRONMENT=0
ROBOT_NUM=$1
SCORE_KEEPING=0
STARTING_PORT_NUM=8000

if [ -z "$ROBOT_NUM" ]
then
      echo "Usage: ./run_single_agent.sh ROBOT_NUM"
      exit
fi

if [ "$ROBOT_NUM" -lt 1 ]
then
      echo "ROBOT_NUM should be a positive number!"
      exit
fi



cd RogerSimulator
./simulator $ENVIRONMENT $ROBOT_NUM 0 &

sleep 1

cd ../RogerProjects
make clean; make

START=1
END=$ROBOT_NUM
for i in $(seq $START $END); do
  let "PORT_NUM= $STARTING_PORT_NUM + $i - 1"

  ./roger 127.0.0.1 $PORT_NUM &
  sleep 1
done

wait