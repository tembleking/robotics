
test-unit:
	poetry install
	poetry run mamba -f documentation -t unit

test-entr:
	find -name "*.py" | entr -c poetry run mamba -f documentation

lint:
	poetry install
	poetry run flake8

push-to-robot:
	poetry build
	-ssh pi@$(shell cat robot_ip) "rm *.whl"
	scp ./dist/robotics-*.whl pi@$(shell cat robot_ip):
	ssh pi@$(shell cat robot_ip) "pip uninstall robotics -y"
	ssh pi@$(shell cat robot_ip) "pip install *.whl"

get-robot-ip:
	nmap 192.168.30.* -sP | \
	grep "Nmap scan report" | \
	cut -d' ' -f5 | \
	xargs -I% sh -c \
		"ssh -o GlobalKnownHostsFile=/dev/null -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no pi@% 'echo %' || :" \
		2>/dev/null > robot_ip

run:
	ssh pi@$(shell cat robot_ip) "bash -c '/home/pi/.local/bin/robot'"

test-ruedas:
	ssh pi@$(shell cat robot_ip) "python test_giro.py"


ssh:
	ssh pi@$(shell cat robot_ip)

show-odometry:
	scp pi@$(shell cat robot_ip):latest_odometry.png /tmp/latest_odometry.png && xdg-open /tmp/latest_odometry.png

show-map:
	scp pi@$(shell cat robot_ip):mapstatus_*.png /tmp/mapstatus.png && xdg-open /tmp/mapstatus.png && ssh pi@$(shell cat robot_ip) 'rm mapstatus_*.png'
