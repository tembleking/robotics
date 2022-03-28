
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
	ssh pi@$(shell cat robot_ip) "rm *.whl"
	scp ./dist/robotics-*.whl pi@$(shell cat robot_ip):
	ssh pi@$(shell cat robot_ip) "pip uninstall robotics -y"
	ssh pi@$(shell cat robot_ip) "pip install *.whl"
