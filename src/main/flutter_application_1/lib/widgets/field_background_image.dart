import 'package:flutter/material.dart';
import 'package:flutter_application_1/utils/constants.dart';

class FieldBackgroundImage extends StatelessWidget {
  final double width;
  final double height;

  const FieldBackgroundImage({
    super.key,
    required this.width,
    required this.height,
  });

  @override
  Widget build(BuildContext context) {
    return Image.asset(
      Constants.fieldImagePath,
      width: width,
      height: height,
      fit: BoxFit.cover,
    );
  }
}