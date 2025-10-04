import 'package:flutter/material.dart';

class Point_selection_state {
  final Offset? firstPoint;
  final Offset? secondPoint;
  final bool isWaitingForSecondPoint;

  const Point_selection_state({
    this.firstPoint,
    this.secondPoint,
    this.isWaitingForSecondPoint = false,
  });

  Point_selection_state copyWith({
    Offset? firstPoint,
    Offset? secondPoint,
    bool? isWaitingForSecondPoint,
  }) {
    return Point_selection_state(
      firstPoint: firstPoint ?? this.firstPoint,
      secondPoint: secondPoint ?? this.secondPoint,
      isWaitingForSecondPoint:
          isWaitingForSecondPoint ?? this.isWaitingForSecondPoint,
    );
  }
}