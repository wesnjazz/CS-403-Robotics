# Runs the roger simulator and a single roger client in the developmental environment

ENVIRONMENT=1
ROBOT_NUM=1


cd RogerSimulator
./simulator $ENVIRONMENT $ROBOT_NUM &

sleep 1


cd ../RogerProjects
make clean; make
./roger 127.0.0.1 8000

