# Geometry assets

`manifest.yaml` names mesh slots that are still primitives. The converted
rover visual is `meshes/cad_visual.obj`, with measurements in
`cad_geometry.json`. That mesh is the Onshape STEP tessellation. It does not
collide. Do not upload team CAD to a conversion service. The SolidWorks zip
is not in this package.

Set `EMBR_CAD_ARCHIVE` only if you already have a local archive whose file name
is `final FULL CAD.zip` (or the same name with `_` or `-`). The simulator does
not search your home directory for it, and it does not load those meshes yet.
