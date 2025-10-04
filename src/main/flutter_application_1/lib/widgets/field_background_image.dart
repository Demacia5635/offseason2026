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
    return MaterialApp(
      home: Scaffold(
        body: Center(
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

void main(){
  runApp(FieldBackgroundImage(width:1000 ,height: 1000 ));
}