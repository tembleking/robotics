
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
	ssh pi@10.1.31.222 "rm *.whl"
	scp ./dist/robotics-*.whl pi@10.1.31.222:
	ssh pi@10.1.31.222 "pip uninstall robotics -y"
	ssh pi@10.1.31.222 "pip install *.whl"
