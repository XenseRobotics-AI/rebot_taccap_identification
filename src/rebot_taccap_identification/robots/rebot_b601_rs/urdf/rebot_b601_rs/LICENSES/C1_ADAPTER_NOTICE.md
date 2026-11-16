# C1 adapter: source and license

The C1 STL geometry and associated adapter data are distributed under MIT,
following the rights owner's choice of the same license and copyright notice
as XTac-UMI-Description: Copyright (c) 2026 Xense Robotics.

License reference: XTac-UMI-Description, LICENSE, revision
604573a0fbd4f49c4c0a4a9b69dc124e62b53946:
https://github.com/XenseRobotics-AI/XTac-UMI-Description/blob/604573a0fbd4f49c4c0a4a9b69dc124e62b53946/LICENSE

## CAD provenance

Source: user-provided `C1-RS-taccap转接件.STEP`, SHA-256 `6cf3d0d1b01c1b0ec6be4c5807bc1e4b9563468ba48b7a0eb53e460302e95f4a`.
SolidWorks 2023 AP203, source millimetres. STEP axes retained; metre STL tessellation
uses 0.005 mm linear and 0.05 rad angular deflection. Original STEP remains an
external build input and is not shipped in wheel resources.

The STEP has no density, material, mass, centre of mass or inertia attributes.
User-supplied SW mass properties are separately preserved at the source repository in provenance/solidworks_mass_properties.json and its
hash-locked screenshot (not included in this package).
The COM tensor is converted from kg*mm^2 to kg*m^2 with negated SW positive
products of inertia. Display rounding and a 0.42 percent SW/STEP volume difference
remain recorded for review; no material, density or hardware accuracy is inferred.
Computed uniform-volume geometric integrals remain separate from physical inputs.

## MIT license

MIT License

Copyright (c) 2026 Xense Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
