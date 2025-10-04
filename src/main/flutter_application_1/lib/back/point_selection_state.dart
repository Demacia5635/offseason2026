import 'package:flutter/material.dart';

class point_selection_state {
  final Offset? firstPoint;
  final Offset? secondPoint;
  final bool isWaitingForSecondPoint;

  const point_selection_state({
    this.firstPoint,
    this.secondPoint,
    this.isWaitingForSecondPoint = false,
  });

  point_selection_state copyWith({
    Offset? firstPoint,
    Offset? secondPoint,
    bool? isWaitingForSecondPoint,
  }) {
    return point_selection_state(
      firstPoint: firstPoint ?? this.firstPoint,
      secondPoint: secondPoint ?? this.secondPoint,
      isWaitingForSecondPoint:
          isWaitingForSecondPoint ?? this.isWaitingForSecondPoint,
    );
  }
}