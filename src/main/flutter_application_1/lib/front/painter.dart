import 'package:flutter/material.dart';
import 'package:flutter_application_1/utils/utils.dart';
import 'package:flutter_application_1/back/robotPaintState.dart';


class painter extends StatelessWidget {
  final double width;
  final double height;
  final Offset? firstPoint;
  final Offset? secondPoint;
  

  const painter({
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
      painter: robotPaintState(
        scale: (Constants.metersToPixelsScale, Constants.metersToPixelsScale),
        startPoint: firstPoint,
        endPoint: secondPoint,
      ),
    );
  }
}