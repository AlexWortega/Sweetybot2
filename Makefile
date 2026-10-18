CAD_PY ?= uv run python
CADGEN ?= uv run cadgen
LEARNING_PY ?= .venv-learning/bin/python
.PHONY: setup setup-learning cad model test simulate render package
setup:
	uv sync --frozen --python 3.13
setup-learning:
	uv venv --python 3.11 .venv-learning
	uv pip install --python .venv-learning/bin/python -r learning/requirements.txt pytest==8.4.2
cad:
	$(CAD_PY) -m scripts.build_cad
	$(CAD_PY) -m scripts.check_cad
	$(CAD_PY) -m scripts.reconcile_mass
	$(CAD_PY) -m scripts.export_link_meshes
	$(CAD_PY) -m scripts.check_assembly
	$(CAD_PY) -m scripts.check_actuation_clearances
	$(CAD_PY) -m scripts.check_new_clearances
model:
	$(LEARNING_PY) -m scripts.generate_descriptions
	$(LEARNING_PY) -m scripts.feasibility
	$(LEARNING_PY) -m scripts.engineering_package
test:
	$(LEARNING_PY) -m pytest -q
	$(CADGEN) urdf validate simulation/generated/sweetie_mini.urdf --strict --json
simulate:
	$(LEARNING_PY) -m scripts.validate_v04
render:
	blender -b --factory-startup --python scripts/render_studio.py -- --out renders/v04
package:
	$(LEARNING_PY) -m scripts.verify_release
	$(LEARNING_PY) -m scripts.package_release
