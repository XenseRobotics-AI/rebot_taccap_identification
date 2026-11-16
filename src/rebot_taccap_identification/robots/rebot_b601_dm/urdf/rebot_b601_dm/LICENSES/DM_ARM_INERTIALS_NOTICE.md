# DM arm nominal inertial parameters

Mass, center of mass and inertia for base_link and link1..5 are taken from
hopcan/reBot-Isaacsim, urdf/reBot_B601_DM/urdf/reBot_B601_DM.urdf.
Repository revision and source SHA-256 are recorded in sources.json.
Geometry and inertia priors are distributed under the source repository's
MIT license, reproduced in REBOT_ISAACSIM_MIT (Copyright 2026 Seeed Studio).
These are simulation CAD priors, not measured identification results.
They replace the previous control-repository priors whose redistribution
license was not established. Masses differ: re-identify and validate on
the actual DM arm before using the new model for gravity compensation.
Joint geometry and the custom massless link6 motor-output frame are unchanged.
