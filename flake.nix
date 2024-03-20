{
  description = "Disciplining algorithm for Atomic Reference Time Card";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/23.11";
  };

  outputs = {
    flake-utils,
    nixpkgs,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      packages.default = pkgs.stdenv.mkDerivation (self: {
        pname = "oscillator-disciplining";
        version = "3.6.2";

        src = ./.;

        buildFlags = ["static"];

        installPhase = ''
          runHook preInstall

          install -Dm644 -t $out/lib lib${self.pname}.a
          mkdir --parents $out/include
          cp --recursive include/${self.pname} $out/include

          runHook postInstall
        '';
      });
    });
}
