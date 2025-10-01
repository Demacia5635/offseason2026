class FieldOverlayPainter extends StatelessWidget {
  final double width;
  final double height;
  final Offset? firstPoint;
  final Offset? secondPoint;

  const FieldOverlayPainter({
    super.key,
    required this.width,
    required this.height,
    this.firstPoint,
    this.secondPoint,
  });

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      size: Size(width, height),
      painter: FieldElementsPainter(
        scale: FieldConstants.metersToPixelsScale,
        startPoint: firstPoint,
        endPoint: secondPoint,
      ),
    );
  }
}