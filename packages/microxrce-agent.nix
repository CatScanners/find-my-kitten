{
  stdenv,
  cmake,
  fetchFromGitHub,
  fast-cdr,
  micro-cdr,
  microxrce-client,
  foonathan-memory,
  spdlog_1_9_2,
  fast-dds,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "micro-xrce-dds-agent;";
  version = "3.0.0";
  src = fetchFromGitHub {
    owner = "eProsima";
    repo = "Micro-XRCE-DDS-Agent";
    tag = "v${finalAttrs.version}";
    hash = "sha256-l4I0FFSW+6mag877o8IP1Kv6DhaUJa7rthlttGEYE8I=";
  };

  # For some reason, it won't accept 3.1.0 otherwise
  preConfigure = ''
    substituteInPlace CMakeLists.txt \
      --replace-fail "set(_fastdds_version 3.1)" "set(_fastdds_version 3.1.0)" \

    substituteInPlace cmake/SuperBuild.cmake \
      --replace-fail "find_package(spdlog \''${_spdlog_version} EXACT QUIET)" \
                     "find_package(spdlog ''${_spdlog_version} QUIET)" \
      --replace-fail "QUIET" ""

  '';

  nativeBuildInputs = [
    cmake
  ];

  buildInputs = [
    foonathan-memory
    fast-cdr
    micro-cdr
    microxrce-client
    fast-dds
    spdlog_1_9_2
  ];
})
