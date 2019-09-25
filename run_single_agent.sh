# Runs the roger simulator and a single roger client

ENVIRONMENT=$1
ROBOT_NUM=1

if [ -z "$ENVIRONMENT" ]
then
      echo "Usage: ./run_single_agent.sh ENVIRONMENT_NUM"
      exit
fi

cd RogerSimulator
./simulator $ENVIRONMENT $ROBOT_NUM &

sleep 1


cd ../RogerProjects
./roger 127.0.0.1 8000

