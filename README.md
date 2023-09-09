# cmsim - A Cortex-M Simulator

I had a need to run a Rust program (well, an OS) which was compiled for an Arm Cortex-M0+ on my Windows/Linux/macOS desktop computers. I could (and do) use QEMU, but none of the QEMU emulated machines have a graphical frame-buffer, which I need. Instead I have been compiling my application to x86 (or Aarch64) and running it natively on my desktop machines, but that is a barrier to loading applications into my OS.

So instead, I figured it couldn't be that hard to write a Cortex-M instruction set simulator. And here we are. It is a work in progress and many instructions are not yet implemented.

## Running

There is a library which implements the simulator, and a binary which takes a binary file as the first command line argument and executes it.

A sample assembly language program is included as [`sample.S`](./sample.S). You build that with [`./makebin.sh`](./makebin.sh).

```console
./makebin.sh
cargo run -- ./target/sample.bin
```

You need to connect to port `127.0.0.1:8000` before the program will run. This
port is for any data received/sent on UART0.

```console
$ nc localhost 8000
```

## Changelog

See [CHANGELOG.md](./CHANGELOG.md)

## Licence

```text
cmsim Copyright (c) Jonathan 'theJPster' Pallant, 2023

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

See the full text in [LICENSE.txt](./LICENSE.txt). Broadly, we (the developers)
interpret this to mean (and note that we are not lawyers and this is not legal
advice) that if you give someone a binary containing this source code (or parts
thereof), you must also give them one of:

* Complete and corresponding source code (e.g. as a link to your **own** on-line
  Git repo) for any GPL components in the state you included them.
* A written offer to provide complete and corresponding source code on
  request.

If you are not offering your binary commercially (i.e. you are not selling it
for commercial gain), and you are using an unmodified upstream version of the
source code, then the third option is to give them:

* A link to the tag/commit-hash on the relevant official Github
  repository - <https://github.com/thejpster/cmsim>.

This is to ensure everyone always has the freedom to access this source code.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you shall be licensed as above,
without any additional terms or conditions.

