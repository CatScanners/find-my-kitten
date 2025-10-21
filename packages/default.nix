{
  pkgs,
  pythonPackages,
}:
let
  callPackage = pkgs.newScope ({ inherit (pkgs) rosPackages; } // packages);
  packages = {
    empy_3_3_4 = pythonPackages.callPackage ./empy.nix { };
    pyros-genmsg = pythonPackages.callPackage ./pyros-genmsg.nix { };
    supervision = pythonPackages.callPackage ./supervision.nix { };

    px4-msgs = callPackage ./px4-msgs.nix { };

    fast-cdr = callPackage ./fast-cdr.nix { };
    micro-cdr = callPackage ./micro-cdr.nix { };
    microxrce-client = callPackage ./microxrce-client.nix { };
    fast-dds = callPackage ./fast-dds.nix { };
    spdlog_1_9_2 = callPackage ./spdlog_1_9_2.nix { };
    microxrce-agent = callPackage ./microxrce-agent.nix { };
  };
in
  packages
