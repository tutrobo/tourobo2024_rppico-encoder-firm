![Build Bynary](https://github.com/tutrobo/tourobo2024_rppico-encoder-firm/actions/workflows/build.yml/badge.svg)
![CI Check](https://github.com/tutrobo/tourobo2024_rppico-encoder-firm/actions/workflows/ci_checks.yml/badge.svg)
# rppico-encoder-node
This firmware transmit cobs encoder data via usb-cdc.
## burn firmware!
1. download __rppico-encoder-node.uf2__ from [latest release](https://github.com/NeyagawaRobocons/rppico-encoder-firm/releases/latest)
2. connect raspberry pi pico with boot mode
3. mount raspberry pi pico mass storage
4. copy rppico-encoder-node.uf2 to raspberry pi pico
5. OK!
## setup enviroment
install toolchain.
```
# install target for arm cortex-M0+
rustup target install thumbv6m-none-eabi
# install tools for development
cargo install flip-link elf2uf2-rs probe-run
```
## build
```
# build
cargo build --release
# create uf2 binary (rppico-encoder-firm is elf file)
elf2uf2rs target/thumbv6m-none-eabi/release/rppico-encoder-firm target/thumbv6m-none-eabi/release/rppico-encoder-firm.uf2
```
## run
you can run program on raspberry pi pico with below command
```
cargo run
```
or
```
cargo build --release
elf2uf2rs -d target/thumbv6m-none-eabi/release/rppico-encoder-firm
```
