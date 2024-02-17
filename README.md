# rppico-encoder-node
This firmware transmit cobs encoder data via usb-cdc.
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
elf2uf2rs target/thumbv6m-none-eabi/release/rppico-encoder-firm
```
