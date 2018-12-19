[![Build Status](https://travis-ci.org/lanl-ansi/ODO.svg?branch=master)](https://travis-ci.org/lanl-ansi/ODO)
[![License](https://img.shields.io/badge/License-BSD--3-brightgreen.svg)](https://opensource.org/licenses/BSD-3-Clause)

<p align="center">
<img src="media/LOGO.jpg">
</p>
<p align="center"> Operations & Design Optimization for Networked Microgrids </p>
<p align="center">Funded by the U.S. Department Of Energy (Program of Dan Ton)</p>

<p align="center">ODO was built using Gravity</p>

<p align="center">
<img src="https://static.wixstatic.com/media/c6cff5_dd7659693c6247dc8eb8605d3dca95e8~mv2_d_3300_2550_s_4_2.png/v1/crop/x_1058,y_575,w_1183,h_1225/fill/w_288,h_298,al_c,usm_0.66_1.00_0.01/c6cff5_dd7659693c6247dc8eb8605d3dca95e8~mv2_d_3300_2550_s_4_2.png" width="150">
</p>
<p align="center">www.allinsights.io/gravity</p>

*****************************
See [INSTALL.md](INSTALL.md) for instructions on running/compiling **ODO**

The **ODO** executable can be found under ODO/bin/
*****************************

# Input

**ODO** takes two different files as input: [NET.json](data_sets/Power/IEEE13.json) and [ODO_INPUT.xlsx](data_sets/Power/ODO_INPUT.xlsx), these can be found under [data_sets/Power](data_sets/Power).

The first file contains all the network properties including branch resistance/reactance, bus shunts, operating limits, and others.

The excel file allows the user to adjust investment options and setup configurations such as runtime limit, maximum number of iterations, and others.

# Running in Docker
Below is a recorded execution of a docker pull and a docker run:

![cover-example](media/docker_pull.gif)

![cover-example](media/Docker_run.gif)

# Licence

This code is provided under a BSD license as part of the Multi-Infrastructure Control and Optimization Toolkit (MICOT) project, LA-CC-13-108.


