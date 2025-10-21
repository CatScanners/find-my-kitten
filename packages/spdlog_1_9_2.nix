{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  fmt_9,
  breakpointHook,
}:
let
  fmt_8 = fmt_9.overrideAttrs (final: old: {
    version = "8.1.1";
    src = fetchFromGitHub {
      inherit (old.src) owner repo;
      tag = final.version;
      hash = "sha256-leb2800CwdZMJRWF5b1Y9ocK0jXpOX/nwo95icDf308=";
    };
    doCheck = false;
  });
in
stdenv.mkDerivation (finalAttrs: {
  pname = "spdlog";
  version = "1.9.2";

  src = fetchFromGitHub {
    owner = "gabime";
    repo = "spdlog";
    rev = "v${finalAttrs.version}";
    hash = "sha256-GSUdHtvV/97RyDKy8i+ticnSlQCubGGWHg4Oo+YAr8Y=";
  };

  nativeBuildInputs = [ cmake breakpointHook ];
  propagatedBuildInputs = [ fmt_8 ];

  cmakeFlags = [
    "-DSPDLOG_BUILD_SHARED=${if stdenv.hostPlatform.isStatic then "OFF" else "ON"}"
    "-DSPDLOG_BUILD_STATIC=${if stdenv.hostPlatform.isStatic then "ON" else "OFF"}"
    "-DSPDLOG_BUILD_EXAMPLE=OFF"
    "-DSPDLOG_BUILD_BENCH=OFF"
    "-DSPDLOG_FMT_EXTERNAL=ON"
    "-DCMAKE_INSTALL_INCLUDEDIR=include"
    "-DCMAKE_INSTALL_LIBDIR=lib"
  ];

  outputs = [
    "out"
    "doc"
  ];
  # spdlog <1.4 is header only, no need to split libraries and headers
  # ++ lib.optional (lib.versionAtLeast finalAttrs.version "1.4") "dev";

  postInstall = ''
    mkdir -p $out/share/doc/spdlog
    cp -rv ../example $out/share/doc/spdlog
  '';

  # doCheck = false;
  # preCheck = "export LD_LIBRARY_PATH=$(pwd)\${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH";

  meta = with lib; {
    description = "Very fast, header only, C++ logging library";
    homepage = "https://github.com/gabime/spdlog";
    license = licenses.mit;
    maintainers = with maintainers; [ obadz ];
    platforms = platforms.all;
  };
})
