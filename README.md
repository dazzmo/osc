<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->

<!-- PROJECT LOGO -->
<br>
<div align="center">

  <h1 align="center">Inverse Dynamics Whole-Body Control</h1>

  <p align="left">
    An inverse-dynamics-based control framework built upon the Pinocchio and CasADi libraries.
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project
<p align="left">
</p>
<p align="right">(<a href="#readme-top">back to top</a>)
</p>

## Getting Started
<a name="getting-started"></a>

### Prerequisites
osc requires the following third-party libraries in order to be built and installed.
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
* [CasADi](https://github.com/casadi/casadi) - This library is responsible for translating the control problems into numerical optimisation problems. As a result, you will need to install any solvers you would like with this library. You can then call them as normal with our solver interface.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation
<a name="installation"></a>

1. Clone the repo
   ```sh
   git clone https://github.com/dazzmo/osc
   ```
2. Build the library
    ```sh
    cd osc
    mkdir build && cd build
    cmake ..
    make
   ```
3. Installation of the library can then be performed by
    ```sh
    make install
    ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the GNU LESSER GENERAL PUBLIC LICENSE License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Damian Abood - damian.abood@sydney.edu.au

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgements
This software package is inspired largely by TSID.
<p align="right">(<a href="#readme-top">back to top</a>)</p>
