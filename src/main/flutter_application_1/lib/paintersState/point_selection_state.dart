import 'package:flutter/material.dart';

class PointSelectionState {
  final Offset? firstPoint;
  final Offset? secondPoint;
  final bool isWaitingForSecondPoint;

  const PointSelectionState({
    this.firstPoint,
    this.secondPoint,
    this.isWaitingForSecondPoint = false,
  });

  PointSelectionState copyWith({
    Offset? firstPoint,
    Offset? secondPoint,
    bool? isWaitingForSecondPoint,
  }) {
    return PointSelectionState(
      firstPoint: firstPoint ?? this.firstPoint,
      secondPoint: secondPoint ?? this.secondPoint,
      isWaitingForSecondPoint:
          isWaitingForSecondPoint ?? this.isWaitingForSecondPoint,
    );
  }
}