# Disciplining Minipod

C disciplining algorithm used with **oscillatord** to discipling oscillators using the [minipod algorithm](https://bitbucket.org/spectracom/minipod/src/master/) developed by Matthias Lorentz.

## Compilation

### Build

```bash
mkdir build && cd build
cmake ..
make
```

### Install locally

``` bash
cd build
sudo make install

```

## Tests

Some tests have been implemented in the test directory:

- **test-utils**: Tests the utils files (linear regression and linear interpolation)
- **test-minipod**:
  - Test od_new_from_config
  - Test od_get_calibration parameters

### Build tests

Test utils:

``` bash
cd test
make test-utils
./test-utils
```

Test minipod:

``` bash
make test-minipod
./test-minipod
```
