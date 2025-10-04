import 'package:flutter/material.dart';
import 'package:flutter_application_1/utils/utils.dart';

class fieldImage extends StatelessWidget {
  final double width;
  final double height;

  const fieldImage({
    super.key,
    required this.width,
    required this.height,
  });

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Center(//cange to to the alll scren
          child: Image.asset(
            Constants.fieldImagePath,
            width: width,
            height: height,
            fit: BoxFit.cover,
          ),
        ),
      ),
    );
  }
}
