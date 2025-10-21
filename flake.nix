{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs =
    {
      nix-ros-overlay,
      nixpkgs,
      ...
    }:
    let
      lib = nixpkgs.lib;
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems =
        func:
        lib.genAttrs systems (
          system:
          func (
            import nixpkgs {
              inherit system;
              overlays = [
                nix-ros-overlay.overlays.default
              ];
              config.permittedInsecurePackages = [
                "freeimage-3.18.0-unstable-2024-04-18"
              ];
            }
          )
        );
    in
    {

      devShells = forAllSystems (
        pkgs:
        let
          python = pkgs.python312;
          pythonPackages = python.pkgs;
          customPkgs = import ./packages { inherit pkgs pythonPackages; };
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              customPkgs.microxrce-agent
              customPkgs.px4-msgs
              colcon
              ninja
              qgroundcontrol
              (python.withPackages (
                p: with p; [
                  kconfiglib
                  jinja2
                  jsonschema
                  pyyaml
                  future
                  lxml
                  libxml2
                  libxslt

                  opencv-python
                  torch
                  numpy
                  pandas
                  yt-dlp
                  ultralytics
                  gitpython
                  urllib3

                  customPkgs.supervision
                  customPkgs.empy_3_3_4
                  customPkgs.pyros-genmsg

                  statsmodels # For ros bag analysis
                  plotly
                  transforms3d
                ]
              ))
              (
                with pkgs.rosPackages.jazzy;
                buildEnv {
                  paths = [
                    ros-core
                    ros-gz-image
                    ros-gz-bridge
                    ros-gz

                    # For rosbag analysis
                    ros2bag
                    rosbag2-storage-sqlite3

                    ament-cmake-core
                    cv-bridge
                    rclcpp
                    rclpy
                  ];
                }
              )
            ];
          };
        }
      );
    };

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
