.PHONY: setup references cad model test simulate render package
setup:
	uv sync --frozen --python 3.13
references:
	uv run python scripts/fetch_references.py
cad:
	uv run python -m scripts.build_cad
	uv run python -m scripts.check_cad
	uv run python -m scripts.check_assembly
	uv run python -m scripts.reconcile_mass
	uv run python -m scripts.export_link_meshes
model:
	uv run python -m scripts.feasibility
	uv run python -m scripts.generate_descriptions
	uv run python -m scripts.engineering_package
test:
	uv run pytest -q
	uv run cadgen urdf validate simulation/generated/sweetie_mini.urdf --strict --json
simulate:
	uv run python -m scripts.validate_simulation
render:
	uv run python -m scripts.render_package
	uv run python -m scripts.render_eyes
package:
	uv run python -m scripts.verify_release
	uv run python -m scripts.package_release
