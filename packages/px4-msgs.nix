{
  stdenv,
  fetchFromGitHub,
  rosPackages,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "px4-msgs";
  version = "1.15.0-unstable-2025-10-22";
  src = fetchFromGitHub {
    owner = "PX4";
    repo = "px4_msgs";
    rev = "49a0f6c52cf86948e46e5df8a7b61e33319c9ed2"; # Latest commit from the release/1.15 branch, not tagged
    hash = "sha256-RQ7xxUmz8KLXCIYo55tHRznbv0SDJlHeBH7hoW+G0qw=";
  };

  nativeBuildInputs = with rosPackages.jazzy; [
    ament-cmake
    builtin-interfaces
    rosidl-default-generators
  ];
})
