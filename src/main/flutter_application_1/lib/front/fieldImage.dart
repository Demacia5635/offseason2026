import 'package:flutter/material.dart';
import 'package:flutter_application_1/utils/utils.dart';

class FieldImage extends StatelessWidget {


  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Image.asset(//cange to to the alll scren
            Constants.fieldImagePath,
            width: double.infinity,
            height: double.infinity,
            fit: BoxFit.cover,
        ),
      ),
    );
  }
}