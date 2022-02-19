
test:
	poetry install
	poetry run mamba -f documentation

test-entr:
	find -name "*.py" | entr -c poetry run mamba -f documentation

lint:
	poetry install
	poetry run flake8
