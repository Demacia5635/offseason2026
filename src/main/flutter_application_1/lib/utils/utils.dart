class Constants {
  static const double robotWidthMeters = 0.8;
  static const double robotHeightMeters = 0.7;
  static const String fieldImagePath = 'assets/frcField2025.jpeg';
  static const double frcFieldSize = 7.9;
  static const double PixelsToMeters= double.infinity/frcFieldSize;
  static const double metersToPixelsScale = 1000/PixelsToMeters;
  static const double robotWidthPixels = robotWidthMeters * PixelsToMeters;
  static const double robotHeightPixels =robotHeightMeters * PixelsToMeters;
}