{
  stdenv,
  cmake,
  fetchFromGitHub,
}:
stdenv.mkDerivation (finalAttrs: {
  pname = "fast-cdr";
  version = "2.2.4";
  src = fetchFromGitHub {
    owner = "eProsima";
    repo = "Fast-CDR";
    tag = "v${finalAttrs.version}";
    hash = "sha256-R+StDJVqT0ktbr4cQBwEAPmju+pmBvxonezsIsPwmgc=";
  };

  nativeBuildInputs = [
    cmake
  ];
})

