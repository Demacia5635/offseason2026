import 'package:flutter/material.dart';

class FieldPainter extends CustomPainter {
  final scale = (0.7, 0.8);
  final Offset? startPoint;
  final Offset? endPoint;

  Fֿ

  FieldElementsPainter({
    required this.scale,
    this.startPoint,
    this.endPoint,
  });

  @override
  void paint(Canvas canvas, Size size) {
    _drawRobot(canvas, size);
    _drawLineBetweenPoints(canvas);
  }

  void _drawRobot(Canvas canvas, Size size) {
    final robotPaint = Paint()
      ..color = Colors.red.withOpacity(0.5)
      ..style = PaintingStyle.fill;

    final robotRect = _calculateRobotPosition(size);
    canvas.drawRect(robotRect, robotPaint);
  }

  Rect _calculateRobotPosition(Size size) {
    final robotWidthPixels = FieldConstants.robotWidthMeters * scale;
    final robotHeightPixels = FieldConstants.robotHeightMeters * scale;

    return Rect.fromLTWH(
      size.width - 2 * scale,
      size.height / 2 - robotHeightPixels / 2,
      robotWidthPixels,
      robotHeightPixels,
    );
  }

  void _drawLineBetweenPoints(Canvas canvas) {
    if (startPoint == null || endPoint == null) return;

    final linePaint = Paint()
      ..color = Colors.blue
      ..strokeWidth = 3;

    canvas.drawLine(startPoint!, endPoint!, linePaint);
  }

  @override
  bool shouldRepaint(covariant FieldElementsPainter oldDelegate) {
    return oldDelegate.startPoint != startPoint ||
        oldDelegate.endPoint != endPoint;
  }
}