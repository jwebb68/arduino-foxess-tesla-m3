{ pkgs ? import <nixpkgs> {} }:

let
  # Pinned nixpkgs, deterministic. Last updated: 2/12/21.
  #pkgs = import (fetchTarball("https://github.com/NixOS/nixpkgs/archive/a58a0b5098f0c2a389ee70eb69422a052982d990.tar.gz")) {};

  # Rolling updates, not deterministic.
  # pkgs = import (fetchTarball("channel:nixpkgs-unstable")) {};
in pkgs.mkShell {
  buildInputs = [
    pkgs.gcc-arm-embedded-7
    pkgs.pkgconfig

    pkgs.astyle
    pkgs.clang-tools
  ];

}
