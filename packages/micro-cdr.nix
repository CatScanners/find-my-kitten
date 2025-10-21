{
  stdenv,
  cmake,
  fetchFromGitHub,
  fast-cdr,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "micro-cdr";
  version = "2.0.1";
  src = fetchFromGitHub {
    owner = "eProsima";
    repo = "Micro-CDR";
    tag = "v${finalAttrs.version}";
    hash = "sha256-X5kE8dMpwXL2hzpT6vY+BHa70Fw21z++vs8nVARpxNk=";
  };

  nativeBuildInputs = [
    cmake
  ];

  buildInputs = [
    fast-cdr
  ];

  postInstall = ''
    ln -s $out/include $out/microcdr-${finalAttrs.version}/include
  '';
})

