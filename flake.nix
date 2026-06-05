{
  description = "PIC32 Cross-Compiler Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };

      tarballPath = builtins.getEnv "PIC32_TARBALL";
      tarball = builtins.path {
        path = tarballPath;
        name = "pic32-64-2017_09_15.tgz";
      };

      pic32-toolchain = pkgs.stdenv.mkDerivation {
        pname = "pic32-toolchain";
        version = "2017.09.15";

        src = tarball;

        nativeBuildInputs = [ pkgs.autoPatchelfHook ];

        buildInputs = [
          pkgs.glibc
          pkgs.zlib
          pkgs.ncurses5
          pkgs.stdenv.cc.cc.lib
        ];

        sourceRoot = ".";

        installPhase = ''
          mkdir -p $out/opt
          cp -r pic32mx $out/opt/pic32mx
        '';

        postFixup = ''
          substituteInPlace $out/opt/pic32mx/bin/pcompile \
            --replace-fail 'PICDIR=/opt/pic32mx' 'PICDIR='${placeholder "out"}'/opt/pic32mx'
        '';
      };
    in
    {
      packages.${system}.pic32-toolchain = pic32-toolchain;

      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [
          pic32-toolchain
          pkgs.gnumake
          pkgs.cmake
          pkgs.git
          pkgs.picocom
          pkgs.minicom
          pkgs.gdb
        ];

        shellHook = ''
          export PATH="${pic32-toolchain}/opt/pic32mx/bin:${pic32-toolchain}/opt/pic32mx/pic32mx/bin:$PATH"
          echo "PIC32 toolchain ready. Try: pic32-gcc --version"
        '';
      };
    };
}
